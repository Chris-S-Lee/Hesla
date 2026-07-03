import hid
import socket
import time
import json
import asyncio
import threading
import websockets

PI_IP = "192.168.0.219"
PI_PORT = 9999

engine_on = False
gear = "P"
led_on = False

current_left = 0.0
current_right = 0.0

ACCEL_RATE = 0.15
DECEL_RATE = 0.25

prev_ignition = False
prev_forward = False
prev_reverse = False
prev_led_btn = False

dist_A, dist_B = 0.0, 0.0

web_data = {
    "engine_on": False,
    "steer": 128,
    "gear": "P",
    "pedal": 0,
    "pedal_type": "N",
    "current_left": 0.0,
    "current_right": 0.0,
    "led_on": False,
    "dist_A": 0.0,
    "dist_B": 0.0
}


async def web_server_handler(websocket):
    global web_data
    try:
        while True:
            await websocket.send(json.dumps(web_data))
            await asyncio.sleep(0.015)
    except websockets.exceptions.ConnectionClosed:
        pass


def start_websocket_server():
    async def main():
        async with websockets.serve(web_server_handler, "0.0.0.0", 8765):
            await asyncio.Future()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(main())
    except Exception as e:
        print(f"\n[웹소켓 서버 에러] {e}")
    finally:
        loop.close()


def parse(data):
    gear_raw = data[0]
    steer = data[3]
    pedal = data[4]
    ignition_raw = data[1]
    led_btn_raw = data[15]

    forward = (gear_raw == 32)
    reverse = (gear_raw == 16)
    ignition = (ignition_raw == 16)
    led_press = (led_btn_raw == 255)

    return steer, pedal, reverse, forward, ignition, led_press


def calc_target_speeds(steer, pedal, gear):
    STEER_FACTOR = 0.85
    BASE_SPEED = 0.75

    if pedal < 128:
        throttle = (128 - pedal) / 128.0 * BASE_SPEED
    else:
        throttle = 0.0

    if gear == "R":
        throttle = -throttle
    elif gear in ("P", "N"):
        throttle = 0.0

    steer_norm = -((steer - 128) / 128.0)

    if steer_norm < 0:
        target_left = throttle * (1.0 + steer_norm * STEER_FACTOR)
        target_right = throttle
    else:
        target_left = throttle
        target_right = throttle * (1.0 - steer_norm * STEER_FACTOR)

    return target_left, target_right


def apply_inertia(current, target):
    if target > current:
        current += ACCEL_RATE
        if current > target:
            current = target
    elif target < current:
        current -= DECEL_RATE
        if current < target:
            current = target
    return current


def send_and_receive(sock, left, right, led_state):
    led_cmd = "LED_ON" if led_state else "LED_OFF"
    msg = f"{left:.2f},{right:.2f},{led_cmd}\n"
    try:
        sock.send(msg.encode())
        resp = sock.recv(1024).decode().strip()
        dA, dB = resp.split(",")
        return float(dA), float(dB)
    except Exception as e:
        print(f"\n[송수신 에러] {e}")
        return 0.0, 0.0


web_thread = threading.Thread(target=start_websocket_server, daemon=True)
web_thread.start()

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(5)

try:
    print(f"📡 {PI_IP}:{PI_PORT}에 연결 시도 중...")
    sock.connect((PI_IP, PI_PORT))
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print("👍 연결 성공!")
except socket.timeout:
    print("❌ 연결 시간 초과! 라즈베리파이 서버를 먼저 켜주세요.")
    raise SystemExit
except Exception as e:
    print(f"❌ 연결 에러: {e}")
    raise SystemExit

try:
    h = hid.device()
    h.open(0x0E8F, 0x0003)
    h.set_nonblocking(1)
except Exception as e:
    print(f"❌ 핸들 연결 실패: {e}")
    sock.close()
    raise SystemExit

print("🎮 아우라 레이싱 휠 HID 원본 패킷 분석 시작!")
print("🌐 대시보드 브로드캐스팅 서버 활성화 [Port: 8765]")

try:
    while True:
        data = h.read(64)
        if not data or len(data) < 18:
            time.sleep(0.01)
            continue

        steer, pedal, reverse, forward, ignition, led_press = parse(data)

        print(
            f"RAW={data[:18]} steer={steer} pedal={pedal} "
            f"ignition={ignition} forward={forward} reverse={reverse} led={led_press}",
            end="\r"
        )
        
        if ignition and not prev_ignition:
            if not engine_on:
                engine_on = True
                gear = "P"
                print("\n🔑 시동 ON / 상태: PARK")
            else:
                engine_on = False
                gear = "P"
                print("\n🔒 시동 OFF / 시스템 정지")

        if engine_on:
            if forward and not prev_forward:
                gear = "D"
                print("\n⬆️ DRIVE 모드 고정")
            elif reverse and not prev_reverse:
                gear = "R"
                print("\n⬇️ REVERSE 모드 고정")


        if led_press and not prev_led_btn:
            led_on = not led_on
            print(f"\n💡 전조등: {'ON' if led_on else 'OFF'}")

        prev_ignition = ignition
        prev_forward = forward
        prev_reverse = reverse
        prev_led_btn = led_press

        if engine_on:
            target_l, target_r = calc_target_speeds(steer, pedal, gear)
            current_left = apply_inertia(current_left, target_l)
            current_right = apply_inertia(current_right, target_r)
        else:
            current_left = apply_inertia(current_left, 0.0)
            current_right = apply_inertia(current_right, 0.0)

        dist_A, dist_B = send_and_receive(sock, current_left, current_right, led_on)

        if pedal < 128:
            pedal_pct = int(((128 - pedal) / 128.0) * 100)
            pedal_type = "A"
        elif pedal > 128:
            pedal_pct = int(((pedal - 128) / 127.0) * 100)
            pedal_type = "B"
        else:
            pedal_pct = 0
            pedal_type = "N"

        web_data["engine_on"] = engine_on
        web_data["steer"] = steer
        web_data["gear"] = gear
        web_data["pedal"] = pedal_pct
        web_data["pedal_type"] = pedal_type
        web_data["current_left"] = round(current_left, 2)
        web_data["current_right"] = round(current_right, 2)
        web_data["led_on"] = led_on
        web_data["dist_A"] = dist_A
        web_data["dist_B"] = dist_B

        print(
            f"\n기어: {gear} | 모터 관성: L{current_left:+.2f}/R{current_right:+.2f} | "
            f"초음파: A:{dist_A:5.1f}cm B:{dist_B:5.1f}cm",
            end="\r"
        )
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n시스템 종료 프로토콜 기동")
    send_and_receive(sock, 0, 0, False)
finally:
    sock.close()
    h.close()
