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
# SlimeVR Destination (UDP)
# ================================================
SLIME_IP = "127.0.0.1"
SLIME_PORT = 6969

# ================================================
# Initial Tracker MAC Addresses (6 Trackers)
# Please configure the 6 unique MAC addresses correctly
# If you don't know your MAC addresses, scan for them using the app tool.
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
bridge_running = False

# ================================================
# BLE Characteristic UUIDs (Mocopi Specs)
# ================================================
DATA_CHAR_UUID = "25047e64-657c-4856-afcf-e315048a965b"
CMD_CHAR_UUID  = "0000ff01-0000-1000-8000-00805f9b34fb"

# ================================================
# Tracker Connection Status Colors
# ================================================
STATUS_COLORS = {
    "Connecting": "blue",
    "Connected": "green",
    "Disconnected": "red"
}

# ================================================
# SlimeVR Packet Generation Functions
# ================================================
def build_handshake(mac, fw, packet_counter):
    """
    Generates a handshake packet in a format recognized by the SlimeVR Server.
    """
    fw_string = f"MoSlime/bleak - Puck Version:{fw}"
    buffer = b'\x00\x00\x00\x03'                   # packet type = 3
    buffer += struct.pack('>Q', packet_counter)    # packet counter
    buffer += struct.pack('>I', 15)                # board type
    buffer += struct.pack('>I', 8)                 # IMU type
    buffer += struct.pack('>I', 7)                 # MCU type
    buffer += struct.pack('>III', 0, 0, 0)         # IMU info
    # Convert version strings to numeric values via simple mapping (e.g., "1.0.0" -> 100)
    buffer += struct.pack('>I', int(fw.replace('.', '')))
    buffer += struct.pack('B', len(fw_string))     # FW文字列長
    buffer += fw_string.encode('UTF-8')            # FW文字列

    # Convert MAC string to 6 bytes
    mac_bytes = bytes.fromhex(mac.replace(':', ''))
    buffer += struct.pack('6s', mac_bytes)

    buffer += struct.pack('B', 255)                # padding
    return buffer

def build_sensor_info(packet_counter, sensor_id=0):
    """
    Sensor info packet for SlimeVR. Trackers display more reliably with this minimum payload.
    Differentiates multiple trackers via the specified sensor_id.
    """
    buffer = b'\x00\x00\x00\x0f'                   # packet type = 15
    buffer += struct.pack('>Q', packet_counter)
    buffer += struct.pack('B', sensor_id)          # tracker (sensor) ID
    buffer += struct.pack('B', 0)                  # sensor status
    buffer += struct.pack('B', 8)                  # sensor type (BMI160 equivalent)
    return buffer

def build_rotation_packet(qw, qx, qy, qz, packet_counter, sensor_id=0):
    """
    Orientation packet (Quaternion) for SlimeVR.
    Packs data using coordinate transformation: x -> -x, y -> z, z -> y.
    Differentiates multiple trackers via the specified sensor_id.
    """
    buffer = b'\x00\x00\x00\x11'                   # packet type = 17
    buffer += struct.pack('>Q', packet_counter)    # packet counter
    buffer += struct.pack('B', sensor_id)          # Sensor ID
    buffer += struct.pack('B', 1)                  # data type (use unknown)
    # Order matched to SlimeVR coordinate spaces: -x, z, y, w
    buffer += struct.pack('>ffff', -qx, qz, qy, qw)
    buffer += struct.pack('B', 0)                  # calibration info
    return buffer

def build_accel_packet(ax, ay, az, packet_counter, sensor_id=0):
    """
    Acceleration packet for SlimeVR.
    Differentiates multiple trackers via the specified sensor_id.
    """
    buffer = b'\x00\x00\x00\x04'                   # packet type = 4
    buffer += struct.pack('>Q', packet_counter)
    buffer += struct.pack('>fff', ax, ay, az)
    buffer += struct.pack('B', sensor_id)          # Sensor ID
    return buffer

# ================================================
# Rotate Quaternion +90° on X-axis & Swap Y/Z
# ================================================
def rotate_quaternion(quat, axis, angle_degrees):
    """
    Returns a new quaternion rotated by a specific angle around a chosen axis.
    """
    angle_radians = math.radians(angle_degrees)
    sin_half = math.sin(angle_radians / 2)
    cos_half = math.cos(angle_radians / 2)

    # Convert rotation axis to a quaternion layout
    w2 = cos_half
    x2 = axis[0] * sin_half
    y2 = axis[1] * sin_half
    z2 = axis[2] * sin_half

    w1, x1, y1, z1 = quat

    # Quaternion multiplication product (Original * Rotation)
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )

# ================================================
# Extract Quaternion & Acceleration from Mocopi Streams
# ================================================
def convert_quaternion_and_accel(data: bytes):
    """
    - data[8:16] is the Quaternion (2 bytes each)
    - data[24:30] is Acceleration (2 bytes each -> float16)
    - Applies custom coordinate fixes for mocopi: Rotates X+90°, swaps Y/Z
    """
    if len(data) < 30:
        return None, None

    # 2 bytes [-8192..8192] -> [-1..1]
    def to_float(b):
        raw = int.from_bytes(b, byteorder='little', signed=True)
        return raw / 8192.0

    # Adjust to standard quaternion order layout: (w, x, -y, z)
    qw = to_float(data[8:10])
    qx = to_float(data[10:12])
    qy = to_float(data[12:14])
    qz = to_float(data[14:16])

    # Equivalent to negating the Y component matching native mocopi processing
    original_quat = (qw, qx, -qy, qz)

    # Apply 90-degree X-axis correction rotation
    rotated_quat = rotate_quaternion(original_quat, (1, 0, 0), 90)

    # Swap Y and Z axes (w, x, z, y)
    final_quat = (
        rotated_quat[0],   # w
        rotated_quat[1],   # x
        rotated_quat[3],   # z -> y
        rotated_quat[2],   # y -> z
    )

    # Unpack acceleration (float16)
    ax = struct.unpack('<e', data[24:26])[0]
    ay = struct.unpack('<e', data[26:28])[0]
    az = struct.unpack('<e', data[28:30])[0]

    return final_quat, (ax, ay, az)

# ================================================
# Single Tracker Instance Connection Manager
# ================================================
class SingleTracker:
    def __init__(self, mac_var, status_var, raw_data_var, canvas, sensor_id=0):
        self.mac_var = mac_var              # MAC address (StringVar)
        self.status_var = status_var        # Connection status (StringVar)
        self.raw_data_var = raw_data_var    # UI display string (StringVar)
        self.canvas = canvas
        self.client = None
        self.pcounter = 0                  # Counter for SlimeVR transmissions
        self.sensor_id = sensor_id         # Unique ID to differentiate trackers
        self.is_running = False

        # Allocate an independent socket for each tracker instance
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2)
        self.sock.bind(('', 0))  # Assign ephemeral port
        self.local_port = self.sock.getsockname()[1]
        print(f"[Tracker {self.sensor_id}] Using local port: {self.local_port}")

    async def disconnect(self):
        """
        BLE Disconnect (Call explicitly to terminate connection)
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
        Transmits the generated data packet to SlimeVRServer (SLIME_IP:SLIME_PORT).
        Because it uses self.sock (independent ports), each tracker is easily treated as a separate process.
        """
        self.sock.sendto(packet, (SLIME_IP, SLIME_PORT))

# ================================================
# Notification Handler
# ================================================
async def notification_handler(_, data: bytes, tracker: SingleTracker):
    """
    Routes incoming raw mocopi sensor telemetry data frames to the SlimeVR server.
    """
    try:
        quat, accel = convert_quaternion_and_accel(data)
        if quat is None or accel is None:
            return

        # Quaternion
        w, x, y, z = quat
        # Acceleration
        ax, ay, az = accel

        # Update packet counter
        tracker.pcounter += 1
        pcounter = tracker.pcounter

        # Transmit Rotation packet
        rot_packet = build_rotation_packet(w, x, y, z, pcounter, sensor_id=tracker.sensor_id)
        tracker.sendto_slimevr(rot_packet)

        # Transmit Accel packet
        tracker.pcounter += 1
        accel_packet = build_accel_packet(ax, ay, az, tracker.pcounter, sensor_id=tracker.sensor_id)
        tracker.sendto_slimevr(accel_packet)

        # Display raw quaternion data strings in the UI panel layout
        tracker.raw_data_var.set(f"W:{w:.2f} X:{x:.2f} Y:{y:.2f} Z:{z:.2f}")

    except Exception as e:
        print(f"Notification Handler Error for {tracker.mac_var.get()}: {e}")

# ================================================
# Update connection status indicator icons
# ================================================
def update_status_icon(canvas, status):
    canvas.delete("all")
    color = STATUS_COLORS.get(status, "red")
    canvas.create_oval(5, 5, 15, 15, fill=color, outline=color)

# ================================================
# Connection maintenance task
# ================================================
async def maintain_connection(tracker: SingleTracker):
    """
    Task to continuously maintain the connection after successfully connecting to a device.
    If an error occurs, execute the disconnection handling procedure.
    """
    print(f"Maintaining connection for {tracker.mac_var.get()} ...")
    try:
        while True:
            await asyncio.sleep(1)
    except Exception as e:
        print(f"maintain_connection error: {e}")
    finally:
        # Disconnect if the task terminates for any reason
        await tracker.disconnect()

# ================================================
# Attempt to connect to a single tracker with up to 3 retry attempts
# If successful, initiate maintain_connection loop and return
# If unsuccessful, preserve Disconnected status parameters
# ================================================
async def connect_tracker_task(idx, mac, trackers):
    """
    Isolated smart-retry worker task. 
    Manages a single tracker's connection loop independently without blocking others.
    """
    global bridge_running
    tracker = trackers[idx]
    
    tracker.is_running = True
    while bridge_running and tracker.is_running:
        if not tracker.client or not tracker.client.is_connected:
            tracker.status_var.set("Connecting")
            update_status_icon(tracker.canvas, "Connecting")
            print(f"[Tracker {idx}] Attempting independent background connection to: {mac}")
            
            try:
                tracker.client = BleakClient(mac, timeout=10.0)
                await tracker.client.connect()
                
                # Windows hardware pairing compatibility layer
                try:
                    if hasattr(tracker.client, "pair"):
                        await tracker.client.pair(protection_level=2)
                        print(f"Pairing completed for {mac}")
                except Exception as pair_err:
                    print(f"Pairing not supported or failed: {pair_err}")
                
                # Transmit initialization handshake packets to SlimeVR
                tracker.sendto_slimevr(build_handshake(mac, "1.0.0", tracker.pcounter))
                tracker.pcounter += 1
                await asyncio.sleep(0.5)
                
                tracker.sendto_slimevr(build_sensor_info(tracker.pcounter, tracker.sensor_id))
                tracker.pcounter += 1
                await asyncio.sleep(0.5)
                
                # Activate notification telemetry streaming
                await tracker.client.start_notify(CHARACTERISTIC_UUID, tracker.notification_handler)
                await tracker.client.write_gatt_char(COMMAND_UUID, bytearray([0x03, 0x00]))
                
                tracker.status_var.set("Connected")
                update_status_icon(tracker.canvas, "Connected")
                print(f"[Tracker {idx}] Connection established successfully.")
                
                # Hand over to connection maintenance tracking state worker
                await maintain_connection(tracker)
                
            except Exception as e:
                print(f"[Tracker {idx}] Connection error encountered: {e}")
                tracker.status_var.set("Disconnected")
                update_status_icon(tracker.canvas, "Disconnected")
                
                try:
                    await tracker.disconnect()
                except:
                    pass
                if not bridge_running or not tracker.is_running:
                    break
                print(f"[Tracker {idx}] Retrying independent connection cycle in 5 seconds...")
                await asyncio.sleep(5.0)
        else:
            await asyncio.sleep(1.0)

    # Clean cleanup execution if the bridge is toggled off
    print(f"[Tracker {idx}] Terminating background task loop...")
    tracker.status_var.set("Disconnected")
    update_status_icon(tracker.canvas, "Disconnected")
    try:
        await tracker.disconnect()
    except:
        pass

# ================================================
# (A) Event loop to handle a single tracker connection layout instance
# ================================================
async def single_main_loop(tracker: SingleTracker):
    # Connect a single tracking unit
    await connect_tracker_once(tracker)

    # Keep the background process active via an infinite loop loop structure
    while True:
        await asyncio.sleep(1)

def start_single_tracker(tracker: SingleTracker):
    """
    Initialize a background thread routine task to handle connecting a single tracking unit.
    """
    import threading
    def background_single():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        idx = tracker.sensor_id
        mac = tracker.mac_var.get()
        loop.run_until_complete(connect_tracker_task(idx, mac, trackers))

    threading.Thread(target=background_single, daemon=True).start()

# =======================================================
# (B) Event loop block tracking automated batch updates array sequences
# =======================================================
async def main_loop(trackers):
    # Launch all tracking unit tasks concurrently in parallel loops
    tasks = []
    for idx, t in enumerate(trackers):
        mac = t.mac_var.get()
        if mac and mac != "00:00:00:00:00:00":
            tasks.append(asyncio.create_task(connect_tracker_task(idx, mac, trackers)))
            
    print(f"Spawned {len(tasks)} parallel background connection threads.")
    
    # Keep the task pool running concurrently forever
    await asyncio.gather(*tasks)

def start_background_loop(trackers):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(main_loop(trackers))

def start_tracker(trackers):
    """
    Initialize a batch background thread connection sequence for all 6 tracking units.
    """
    import threading
    threading.Thread(target=start_background_loop, args=(trackers,), daemon=True).start()

# ================================================
# Bluetooth Device Scanner Routine
# ================================================
def scan_bt_devices_thread(mac_list_var: tk.StringVar):
    async def async_scan():
        mac_list = []
        mac_list_var.set("Scanning...")
        try:
            devices = await BleakScanner.discover(timeout=10.0)  # Set scan time to 10 seconds
            for device in devices:
                # Filter and extract specific devices starting with the "QM-SS1" hardware indicator naming layout
                if device.name and device.name.startswith("QM-SS1"):
                    mac_list.append(f"{device.name}: {device.address}")
        except Exception as e:
            mac_list_var.set(f"Scan Error: {e}")
            return

        # Restrict layout capacity to display a maximum of 10 discovered targets
        if mac_list:
            mac_list_var.set("\n".join(mac_list[:10]))
        else:
            mac_list_var.set("No Trackers Found")

    asyncio.run(async_scan())

def on_scan():
    import threading
    threading.Thread(target=scan_bt_devices_thread, args=(mac_list_var,), daemon=True).start()

def on_copy():
    try:
        # Filter MAC addresses starting with "3C:" from the scan list
        mac_addresses = [item.split(": ")[-1] for item in mac_listbox.get(0, tk.END) if item.split(": ")[-1].startswith("3C:")]
        if not mac_addresses:
            messagebox.showwarning("Warning", "No MAC addresses starting with '3C:' found in scan results.")
            return

        # Populate input fields with MAC addresses regardless of order
        for i, mac in enumerate(mac_addresses):
            if i < len(trackers): # Ensure count doesn't exceed total tracker count
                trackers[i].mac_var.set(mac)

        messagebox.showinfo("Copy Complete", f"Successfully populated input fields with {len(mac_addresses)} MAC addresses starting with '3C:'.")
    except Exception as e:
        messagebox.showwarning("Error", f"An unexpected error occurred: {e}")

# ================================================
# Save & Load Configuration Settings
# ================================================
def save_config(trackers):
    config = {
        "trackers": [t.mac_var.get() for t in trackers]
    }
    with open(CONFIG_FILE, "w") as f:
        json.dump(config, f)
    messagebox.showinfo("Success", "Configuration settings saved successfully!")

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
        # Load default address array parameters
        for i, t in enumerate(trackers):
            t.mac_var.set(DEFAULT_MAC_ADDRESSES[i])

# ================================================
# GUI Construction
# ================================================
root = tk.Tk()
root.title("SlimeVR Multi Bridge (6 Trackers)")
root.geometry("1000x950")
root.configure(bg="#1e1e1e")

# Fix window resolution configuration settings
root.resizable(False, False)  # Horizontal(False), Vertical(False)

# Custom Style Configuration
style = ttk.Style()
style.configure("TLabel", background="#1e1e1e", foreground="#ffffff", font=("Arial", 11))
style.configure("TButton", background="#ffffff", foreground="#000000", font=("Arial", 12, "bold"))
style.configure("TEntry", font=("Arial", 11))

# Variable for displaying scan results
mac_list_var = tk.StringVar(value="No Scan Results")

# Create tracker instances array
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

        single_tracker_button = ttk.Button(
            root,
            text="Start Single",
            command=lambda t=i: start_single_tracker(trackers[t])
        )
        single_tracker_button.grid(row=row_base+1, column=3, padx=(10, 2), pady=5, sticky="e")

        single_stop_button = ttk.Button(
            root,
            text="Stop Single",
            command=lambda t=i: on_stop_single(t)
        )
        single_stop_button.grid(row=row_base+1, column=4, padx=(2, 10), pady=5, sticky="w")

        tracker = SingleTracker(mac_var, status_var, raw_data_var, canvas, sensor_id=i)
        trackers.append(tracker)

# # Scan Results Listbox
listbox_label_row = 12
ttk.Label(root, text="Scan Results:").grid(row=listbox_label_row, column=0, pady=10, padx=10, sticky="ne")
mac_listbox = tk.Listbox(root, listvariable=mac_list_var, height=15, width=30, bg="#2e2e2e", fg="#ffffff", selectbackground="#444444")
mac_listbox.grid(row=listbox_label_row, column=1, columnspan=3, pady=10, padx=10, sticky="w")

button_frame = tk.Frame(root, bg="#1e1e1e")
button_frame.grid(row=listbox_label_row+1, column=1, columnspan=3, pady=10, padx=10, sticky="w")

ttk.Button(button_frame, text="Scan MACs", command=on_scan).grid(row=0, column=0, padx=10)
ttk.Button(button_frame, text="Copy", command=on_copy).grid(row=0, column=1, padx=10)

def on_clear_scan_log():
    mac_list_var.set("")
    print("Scan results terminal log cleared.")

ttk.Button(button_frame, text="Clear Log", command=on_clear_scan_log).grid(row=1, column=0, padx=10, pady=5)

def on_start_all():
    global bridge_running
    bridge_running = True  # Arm the master loop flag
    start_tracker(trackers)

def on_stop_all():
    global bridge_running
    print("Stopping all tracking bridge connection tasks...")
    bridge_running = False  # Tripping this flag breaks active loops instantly

ttk.Button(button_frame, text="Start Bridge (All)", command=on_start_all).grid(row=0, column=2, padx=10)
ttk.Button(button_frame, text="Stop Bridge", command=on_stop_all).grid(row=1, column=2, padx=10, pady=5)
ttk.Button(button_frame, text="Save Settings", command=lambda: save_config(trackers)).grid(row=0, column=3, padx=10)
ttk.Button(button_frame, text="Load Settings", command=lambda: load_config(trackers)).grid(row=0, column=4, padx=10)

load_config(trackers)
root.mainloop()
