import asyncio
import struct
import socket
import math
import tkinter as tk
from tkinter import ttk, messagebox
import json
from bleak import BleakClient
from bleak import BleakScanner
import random

# ================================================
# SlimeVR 送信先（UDP）
# ================================================
SLIME_IP = "127.0.0.1"
SLIME_PORT = 6969

# ================================================
# トラッカーのMACアドレス初期値（6トラッカー）
# 6つの異なるMACを正しく設定してください
# 実機でMACアドレスが分からない場合は、適宜書き換えてお使いください
# ================================================
DEFAULT_MAC_ADDRESSES = [
    "3C:38:F4:B4:95:01",
    "3C:38:F4:B4:95:02",
    "3C:38:F4:B4:95:03",
    "3C:38:F4:B4:95:04",
    "3C:38:F4:B4:95:05",
    "3C:38:F4:B4:95:06",
]

CONFIG_FILE = "config_six.json"

# ================================================
# BLEキャラクタリスティックUUID (Mocopi想定)
# ================================================
DATA_CHAR_UUID = "25047e64-657c-4856-afcf-e315048a965b"
CMD_CHAR_UUID  = "0000ff01-0000-1000-8000-00805f9b34fb"

# ================================================
# 接続状態の色設定
# ================================================
STATUS_COLORS = {
    "Connecting": "blue",
    "Connected": "green",
    "Disconnected": "red"
}

# ================================================
# SlimeVRパケット生成関数
# ================================================
def build_handshake(mac, fw, packet_counter):
    """
    SlimeVRサーバが認識する形式のハンドシェイクパケットを生成
    """
    fw_string = f"MoSlime/bleak - Puck Version:{fw}"
    buffer = b'\x00\x00\x00\x03'                   # packet type = 3
    buffer += struct.pack('>Q', packet_counter)    # packet counter
    buffer += struct.pack('>I', 15)                # board type
    buffer += struct.pack('>I', 8)                 # IMU type
    buffer += struct.pack('>I', 7)                 # MCU type
    buffer += struct.pack('>III', 0, 0, 0)         # IMU info
    # "1.0.0" -> 100 などバージョン文字列を数字化(単純マッピング)
    buffer += struct.pack('>I', int(fw.replace('.', '')))
    buffer += struct.pack('B', len(fw_string))     # FW文字列長
    buffer += fw_string.encode('UTF-8')            # FW文字列

    # MACを6バイトに変換
    mac_bytes = bytes.fromhex(mac.replace(':', ''))
    buffer += struct.pack('6s', mac_bytes)

    buffer += struct.pack('B', 255)                # padding
    return buffer

def build_sensor_info(packet_counter, sensor_id=0):
    """
    SlimeVR用のセンサ情報パケット。最低限これがあるとトラッカーが表示されやすい
    sensor_idを指定して複数トラッカーを区別できる
    """
    buffer = b'\x00\x00\x00\x0f'                   # packet type = 15
    buffer += struct.pack('>Q', packet_counter)
    buffer += struct.pack('B', sensor_id)          # tracker (sensor) ID
    buffer += struct.pack('B', 0)                  # sensor status
    buffer += struct.pack('B', 8)                  # sensor type (BMI160相当)
    return buffer

def build_rotation_packet(qw, qx, qy, qz, packet_counter, sensor_id=0):
    """
    SlimeVR用の姿勢パケット(クォータニオン)
    x→-x, y→z, z→y の変換でパッキングする
    sensor_idを指定して複数トラッカーを区別できる
    """
    buffer = b'\x00\x00\x00\x11'                   # packet type = 17
    buffer += struct.pack('>Q', packet_counter)    # packet counter
    buffer += struct.pack('B', sensor_id)          # センサID
    buffer += struct.pack('B', 1)                  # data type (use unknown)
    # SlimeVR座標系に合わせて -x, z, y, w の順
    buffer += struct.pack('>ffff', -qx, qz, qy, qw)
    buffer += struct.pack('B', 0)                  # calibration info
    return buffer

def build_accel_packet(ax, ay, az, packet_counter, sensor_id=0):
    """
    SlimeVR用の加速度パケット
    sensor_idを指定して複数トラッカーを区別できる
    """
    buffer = b'\x00\x00\x00\x04'                   # packet type = 4
    buffer += struct.pack('>Q', packet_counter)
    buffer += struct.pack('>fff', ax, ay, az)
    buffer += struct.pack('B', sensor_id)          # センサID
    return buffer

# ================================================
# QuaternionをX軸+90°回転してYとZを入れ替える
# ================================================
def rotate_quaternion(quat, axis, angle_degrees):
    """
    クォータニオン quat=(w, x, y, z) を指定角度だけ回転させた新クォータニオンを返す
    """
    angle_radians = math.radians(angle_degrees)
    sin_half = math.sin(angle_radians / 2)
    cos_half = math.cos(angle_radians / 2)

    # 回転軸をクォータニオンに
    w2 = cos_half
    x2 = axis[0] * sin_half
    y2 = axis[1] * sin_half
    z2 = axis[2] * sin_half

    w1, x1, y1, z1 = quat

    # クォータニオンの積 (元 * 回転)
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )

# ================================================
# mocopiっぽいクォータニオン・加速度の抽出
# ================================================
def convert_quaternion_and_accel(data: bytes):
    """
    - data[8:16] がクォータニオン (各2byte)
    - data[24:30] が加速度 (各2byte → float16)
    - mocopi流の座標補正後: X軸+90°回転、YとZ入れ替え
    """
    if len(data) < 30:
        return None, None

    # 2バイト[-8192..8192]->[-1..1]
    def to_float(b):
        raw = int.from_bytes(b, byteorder='little', signed=True)
        return raw / 8192.0

    # クォータニオン (w, x, -y, z) の順に修正
    qw = to_float(data[8:10])
    qx = to_float(data[10:12])
    qy = to_float(data[12:14])
    qz = to_float(data[14:16])

    # mocopiの "Yに-を付与" 相当
    original_quat = (qw, qx, -qy, qz)

    # X軸 +90度回転
    rotated_quat = rotate_quaternion(original_quat, (1, 0, 0), 90)

    # Y軸とZ軸を入れ替え (w, x, z, y)
    final_quat = (
        rotated_quat[0],   # w
        rotated_quat[1],   # x
        rotated_quat[3],   # z -> y
        rotated_quat[2],   # y -> z
    )

    # 加速度 (float16) unpack
    ax = struct.unpack('<e', data[24:26])[0]
    ay = struct.unpack('<e', data[26:28])[0]
    az = struct.unpack('<e', data[28:30])[0]

    return final_quat, (ax, ay, az)

# ================================================
# 1台用トラッカー管理クラス
# ================================================
class SingleTracker:
    def __init__(self, mac_var, status_var, raw_data_var, canvas, sensor_id=0):
        self.mac_var = mac_var              # MACアドレス (StringVar)
        self.status_var = status_var        # 接続状態 (StringVar)
        self.raw_data_var = raw_data_var    # UI表示用文字列 (StringVar)
        self.canvas = canvas
        self.client = None
        self.pcounter = 0                  # SlimeVR送信用カウンタ
        self.sensor_id = sensor_id         # トラッカーを区別するためのID

        # それぞれ独立したソケットを確保する
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2)
        self.sock.bind(('', 0))  # エフェメラルポート割り当て
        self.local_port = self.sock.getsockname()[1]
        print(f"[Tracker {self.sensor_id}] Using local port: {self.local_port}")

    async def disconnect(self):
        """
        BLE切断 (明示的に呼び出して接続停止)
        """
        if self.client and self.client.is_connected:
            try:
                await self.client.disconnect()
                print(f"Disconnected from tracker {self.mac_var.get()}")
            except Exception as e:
                print(f"Error disconnecting from tracker {self.mac_var.get()}: {e}")
        self.status_var.set("Disconnected")
        update_status_icon(self.canvas, "Disconnected")

    def sendto_slimevr(self, packet: bytes):
        """
        生成したパケットを SlimeVRServer (SLIME_IP:SLIME_PORT) に送信
        self.sock (独立ポート) を使うため、各トラッカーが“別プロセス”扱いされやすい
        """
        self.sock.sendto(packet, (SLIME_IP, SLIME_PORT))

# ================================================
# 通知ハンドラー
# ================================================
async def notification_handler(_, data: bytes, tracker: SingleTracker):
    """
    mocopiデータをSlimeVRサーバに送信する
    """
    try:
        quat, accel = convert_quaternion_and_accel(data)
        if quat is None or accel is None:
            return

        # クォータニオン
        w, x, y, z = quat
        # 加速度
        ax, ay, az = accel

        # パケットカウンタ更新
        tracker.pcounter += 1
        pcounter = tracker.pcounter

        # Rotationパケット送信
        rot_packet = build_rotation_packet(w, x, y, z, pcounter, sensor_id=tracker.sensor_id)
        tracker.sendto_slimevr(rot_packet)

        # Accelパケット送信
        tracker.pcounter += 1
        accel_packet = build_accel_packet(ax, ay, az, tracker.pcounter, sensor_id=tracker.sensor_id)
        tracker.sendto_slimevr(accel_packet)

        # UIに生クォータニオンを表示
        tracker.raw_data_var.set(f"W:{w:.2f} X:{x:.2f} Y:{y:.2f} Z:{z:.2f}")

    except Exception as e:
        print(f"Notification Handler Error for {tracker.mac_var.get()}: {e}")

# ================================================
# 接続状態のアイコン更新
# ================================================
def update_status_icon(canvas, status):
    canvas.delete("all")
    color = STATUS_COLORS.get(status, "red")
    canvas.create_oval(5, 5, 15, 15, fill=color, outline=color)

# ================================================
# 接続維持用タスク
# ================================================
async def maintain_connection(tracker: SingleTracker):
    """
    デバイスに接続後、ずっと維持し続けるためのタスク。
    エラーが発生したら切断処理を行う。
    """
    print(f"Maintaining connection for {tracker.mac_var.get()} ...")
    try:
        while True:
            await asyncio.sleep(1)
    except Exception as e:
        print(f"maintain_connection error: {e}")
    finally:
        # 何らかの理由でタスクが終わったときは切断
        await tracker.disconnect()

# ================================================
# 1台のトラッカーを最大3回リトライして接続
# 成功したら maintain_connection を開始して return
# 失敗したら Disconnected のまま
# ================================================
async def connect_tracker_once(tracker: SingleTracker):
    max_retries = 3
    attempt = 0

    while attempt < max_retries:
        attempt += 1
        tracker.status_var.set("Connecting")
        update_status_icon(tracker.canvas, "Connecting")

        try:
            # 接続タイムアウトを短め(10s)に設定
            tracker.client = BleakClient(tracker.mac_var.get(), timeout=10.0)
            await tracker.client.connect()

            # ----------------------------------------------------
            # 可能であればペアリングを試みる (Windows,一部環境のみ有効)
            # ----------------------------------------------------
            try:
                if hasattr(tracker.client, "pair"):
                    await tracker.client.pair(protection_level=2)
                    print(f"Pairing attempted for {tracker.mac_var.get()}")
            except Exception as pair_err:
                print(f"Pairing not supported or failed: {pair_err}")

            # 接続成功
            tracker.status_var.set("Connected")
            update_status_icon(tracker.canvas, "Connected")
            print(f"[Attempt {attempt}] Connected to {tracker.mac_var.get()} (Sensor ID={tracker.sensor_id})")

            # SlimeVR用 ハンドシェイクパケット送信
            fw_ver = "1.0.0"
            tracker.pcounter += 1
            handshake = build_handshake(tracker.mac_var.get(), fw_ver, tracker.pcounter)
            tracker.sendto_slimevr(handshake)

            # 少し待機してからセンサ情報
            await asyncio.sleep(0.3)

            tracker.pcounter += 1
            sensor_info = build_sensor_info(tracker.pcounter, sensor_id=tracker.sensor_id)
            tracker.sendto_slimevr(sensor_info)

            # BLE通知購読開始
            await tracker.client.start_notify(
                DATA_CHAR_UUID,
                lambda s, d: asyncio.create_task(notification_handler(s, d, tracker))
            )
            print(f"[{tracker.mac_var.get()}] Notification started.")

            # Mocopiストリーミング開始コマンド
            await tracker.client.write_gatt_char(
                CMD_CHAR_UUID,
                bytearray([0x7e, 0x03, 0x18, 0xd6, 0x01, 0x00, 0x00])
            )
            print(f"[{tracker.mac_var.get()}] Stream start command sent.")

            # 接続維持タスクを起動 (ノンブロッキング)
            asyncio.create_task(maintain_connection(tracker))

            # 接続が成功したらreturn
            return

        except Exception as e:
            tracker.status_var.set("Disconnected")
            update_status_icon(tracker.canvas, "Disconnected")
            print(f"[Attempt {attempt}] Error connecting to {tracker.mac_var.get()} (ID={tracker.sensor_id}): {e}")
            if attempt < max_retries:
                print(f"Retrying... (max {max_retries} attempts)")
                await asyncio.sleep(2)  # 次の接続まで少し待機
            else:
                print(f"Failed to connect after {max_retries} attempts: {tracker.mac_var.get()}")

    # 失敗してここに来たら諦める
    tracker.status_var.set("Disconnected")
    update_status_icon(tracker.canvas, "Disconnected")

# ================================================
# (A) 1台を単独接続するイベントループ
# ================================================
async def single_main_loop(tracker: SingleTracker):
    # 1台だけ接続
    await connect_tracker_once(tracker)

    # ここで無限ループして終了させない
    while True:
        await asyncio.sleep(1)

def start_single_tracker(tracker: SingleTracker):
    """
    指定トラッカーだけ接続を試みるスレッドを起動
    """
    import threading
    def background_single():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(single_main_loop(tracker))

    threading.Thread(target=background_single, daemon=True).start()

# ================================================
# (B) 複数台を順番に接続するイベントループ
# ================================================
async def main_loop(trackers):
    for t in trackers:
        await connect_tracker_once(t)
    print("All trackers processed.")
    # ここで無限ループして維持
    while True:
        await asyncio.sleep(1)

def start_background_loop(trackers):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(main_loop(trackers))

def start_tracker(trackers):
    """
    6台一括接続開始
    """
    import threading
    threading.Thread(target=start_background_loop, args=(trackers,), daemon=True).start()

# ================================================
# BTデバイススキャン関連
# ================================================
def scan_bt_devices_thread(mac_list_var: tk.StringVar):
    async def async_scan():
        mac_list = []
        mac_list_var.set("スキャン中...")
        try:
            devices = await BleakScanner.discover(timeout=10.0)  # スキャン時間を20秒に設定
            for device in devices:
                # 例として"QM-SS1"で始まる名前を表示
                if device.name and device.name.startswith("QM-SS1"):
                    mac_list.append(f"{device.name}: {device.address}")
        except Exception as e:
            mac_list_var.set(f"スキャンエラー: {e}")
            return

        # 最大10台まで表示
        if mac_list:
            mac_list_var.set("\n".join(mac_list[:10]))
        else:
            mac_list_var.set("該当デバイスなし")

    asyncio.run(async_scan())

def on_scan():
    import threading
    threading.Thread(target=scan_bt_devices_thread, args=(mac_list_var,), daemon=True).start()

def on_copy():
    try:
        # スキャンリストから "3C:" で始まるMACアドレスをフィルタリング
        mac_addresses = [item.split(": ")[-1] for item in mac_listbox.get(0, tk.END) if item.split(": ")[-1].startswith("3C:")]
        if not mac_addresses:
            messagebox.showwarning("警告", "スキャン結果に '3C:' で始まるMACアドレスがありません")
            return

        # MACアドレスを順不同で各入力欄に設定
        for i, mac in enumerate(mac_addresses):
            if i < len(trackers):  # トラッカー数を超えないように設定
                trackers[i].mac_var.set(mac)

        messagebox.showinfo("コピー", f"'3C:' で始まるMACアドレス {len(mac_addresses)} 件を入力欄に設定しました")
    except Exception as e:
        messagebox.showwarning("警告", f"エラーが発生しました: {e}")


# ================================================
# 設定の保存・読込
# ================================================
def save_config(trackers):
    config = {
        "trackers": [t.mac_var.get() for t in trackers]
    }
    with open(CONFIG_FILE, "w") as f:
        json.dump(config, f)
    messagebox.showinfo("保存", "設定を保存しました")

def load_config(trackers):
    try:
        with open(CONFIG_FILE, "r") as f:
            config = json.load(f)
            loaded = config.get("trackers", DEFAULT_MAC_ADDRESSES)
            for i, t in enumerate(trackers):
                if i < len(loaded):
                    t.mac_var.set(loaded[i])
                else:
                    t.mac_var.set(DEFAULT_MAC_ADDRESSES[i])
    except FileNotFoundError:
        # デフォルトアドレスをロード
        for i, t in enumerate(trackers):
            t.mac_var.set(DEFAULT_MAC_ADDRESSES[i])

# ================================================
# GUI構築
# ================================================
root = tk.Tk()
root.title("SlimeVR Multi Bridge (6トラッカー)")
root.geometry("1000x950")
root.configure(bg="#1e1e1e")

# ウィンドウの解像度を固定
root.resizable(False, False)  # 横方向(False), 縦方向(False)

# カスタムスタイル
style = ttk.Style()
style.configure("TLabel", background="#1e1e1e", foreground="#ffffff", font=("Arial", 11))
style.configure("TButton", background="#ffffff", foreground="#000000", font=("Arial", 12, "bold"))
style.configure("TEntry", font=("Arial", 11))

# スキャン結果表示用の変数
mac_list_var = tk.StringVar(value="スキャン結果なし")

# トラッカー配列作成
trackers = []

for i in range(6):
    row_base = i * 2

    ttk.Label(root, text=f"Tracker #{i+1} MAC:").grid(row=row_base, column=0, pady=10, padx=10, sticky="e")

    mac_var = tk.StringVar()
    entry = ttk.Entry(root, textvariable=mac_var, width=30)
    entry.grid(row=row_base, column=1, columnspan=3, pady=10, padx=10, sticky="w")

    canvas = tk.Canvas(root, width=20, height=20, bg="#1e1e1e", highlightthickness=0)
    canvas.grid(row=row_base+1, column=0, pady=5, padx=10, sticky="e")
    update_status_icon(canvas, "Disconnected")

    status_var = tk.StringVar(value="Disconnected")
    ttk.Label(root, textvariable=status_var).grid(row=row_base+1, column=1, pady=5, padx=10, sticky="w")

    raw_data_var = tk.StringVar(value="N/A")
    ttk.Label(root, textvariable=raw_data_var).grid(row=row_base+1, column=2, padx=10, pady=5, sticky="w")

    # **単体ブリッジ開始ボタン** を追加
    # command でトラッカーを束縛するために lambda t=tracker: ... の形を使う
    single_tracker_button = ttk.Button(
        root, 
        text="単体開始", 
        command=lambda t=i: start_single_tracker(trackers[t])
    )
    single_tracker_button.grid(row=row_base+1, column=3, padx=10, pady=5, sticky="w")

    tracker = SingleTracker(mac_var, status_var, raw_data_var, canvas, sensor_id=i)
    trackers.append(tracker)

# スキャン結果リストボックス
listbox_label_row = 12
ttk.Label(root, text="スキャン結果:").grid(row=listbox_label_row, column=0, pady=10, padx=10, sticky="ne")
mac_listbox = tk.Listbox(root, listvariable=mac_list_var, height=15, width=30,
                         bg="#2e2e2e", fg="#ffffff", selectbackground="#444444")
mac_listbox.grid(row=listbox_label_row, column=1, columnspan=3, pady=10, padx=10, sticky="w")

button_frame = tk.Frame(root, bg="#1e1e1e")
button_frame.grid(row=listbox_label_row+1, column=1, columnspan=3, pady=10, padx=10, sticky="w")

ttk.Button(button_frame, text="MACスキャン", command=on_scan).grid(row=0, column=0, padx=10)
ttk.Button(button_frame, text="コピー", command=on_copy).grid(row=0, column=1, padx=10)

def on_start_all():
    start_tracker(trackers)

ttk.Button(button_frame, text="ブリッジ開始(一括)", command=on_start_all).grid(row=0, column=2, padx=10)
ttk.Button(button_frame, text="設定保存", command=lambda: save_config(trackers)).grid(row=0, column=3, padx=10)
ttk.Button(button_frame, text="設定読込", command=lambda: load_config(trackers)).grid(row=0, column=4, padx=10)

load_config(trackers)
root.mainloop()
