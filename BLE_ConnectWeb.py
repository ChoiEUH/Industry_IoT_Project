import asyncio
import threading
from bleak import BleakScanner, BleakClient
from flask import Flask, render_template_string
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app, async_mode="threading")

HM10_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
buffer = ""

HTML_PAGE = """
<!doctype html>
<html>
<head>
    <title>HM10 실시간 데이터</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.5.4/socket.io.js"></script>
</head>
<body>
    <h2>📡 HM10 실시간 데이터</h2>
    <pre id="log"></pre>

    <script>
        var socket = io();
        socket.on("hm10_data", function(data) {
            document.getElementById("log").innerText += data + "\\n";
        });
    </script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


def notification_handler(sender, data):
    global buffer
    try:
        text = data.decode("utf-8")
        buffer += text
        while "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            print(f"[HM10] {line.strip()}")
            socketio.emit("hm10_data", line.strip())
    except UnicodeDecodeError:
        print(f"[HM10-RAW] {data}")


async def run_ble():
    print("BLE 장치 스캔 중...")
    devices = await BleakScanner.discover()

    target = None
    for d in devices:
        print(f"발견: {d.name} [{d.address}]")
        if d.name and ("BT05" in d.name or "HMSoft" in d.name):
            target = d
            break

    if not target:
        print("HM10 장치를 찾지 못했습니다.")
        return

    print(f"HM10 장치 발견: {target.name} ({target.address})")

    async with BleakClient(target.address) as client:
        print("연결 시도")
        if client.is_connected:
            print("HM10 연결 성공")
            await client.start_notify(HM10_CHAR_UUID, notification_handler)
            while True:
                await asyncio.sleep(1)

def start_ble_loop():
    asyncio.run(run_ble())

if __name__ == "__main__":
    threading.Thread(target=start_ble_loop, daemon=True).start()
    print("Flask 서버 실행: http://127.0.0.1:5000")
    socketio.run(app, host="0.0.0.0", port=5000)
