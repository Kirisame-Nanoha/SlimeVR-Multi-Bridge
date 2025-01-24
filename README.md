# SlimeVR-Multi-Bridge
MoslimeベースのWindows向けMocopiのSlimeVRブリッジアプリ

BluepyはLinuxでしか利用できないのでBleakを使ってMocopiの直接続を出来るツールを作成しました。
VScodeにてPython3.11を使って作成しました
仮想環境を有効にしてBleakをインストールして下さい
 .\Scripts\Activate.ps1
pip install bleak

exeにする場合は以下を入れます
pip install pyinstaller
pip install winrt
