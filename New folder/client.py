import hid
import socket
import time
import json
import asyncio
import threading
import websockets

PI_IP = "192.168.0.219"
PI_PORT = 9999

WHEEL_VID = 0x0E8F
WHEEL_PID = 0x0003

engine_on = False
gear = "PARK"
led_on = False

current_left = 0.0
current_right = 0.0

USE_INERTIA = False

# 주행 / 회전 설정
BASE_SPEED = 1.00
STEER_DEADZONE = 0.03
PIVOT_START = 0.22
PIVOT_POWER_MIN = 0.55
PIVOT_POWER_GAIN = 1.00

prev_ignition = False
prev_forward = False
prev_reverse = False

# LED 버튼은 아직 실제 매핑 확인 전까지 비활성화
ENABLE_LED_TOGGLE = False
prev_led_btn = False

dist_A, dist_B = 0.0, 0.0
last_error = ""
pi_connected = False
wheel_connected = False

web_data = {
    "engine_on": False,
    "steer": 128,
    "gear": "PARK",
    "pedal": 0,
    "pedal_type": "N",
    "current_left": 0.0,
    "current_right": 0.0,
    "led_on": False,
    "dist_A": 0.0,
    "dist_B": 0.0,
    "pi_connected": False,
    "wheel_connected": False,
    "last_error": "",
    "camera_mode": "front_fixed"
}


def update_status():
    web_data["engine_on"] = engine_on
    web_data["gear"] = gear
    web_data["current_left"] = round(current_left, 2)
    web_data["current_right"] = round(current_right, 2)
    web_data["led_on"] = led_on
    web_data["dist_A"] = dist_A
    web_data["dist_B"] = dist_B
    web_data["pi_connected"] = pi_connected
    web_data["wheel_connected"] = wheel_connected
    web_data["last_error"] = last_error


async def web_server_handler(websocket):
    try:
        while True:
            await websocket.send(json.dumps(web_data))
            await asyncio.sleep(0.03)
    except websockets.exceptions.ConnectionClosed:
        pass


def start_websocket_server():
    async def main():
        async with websockets.serve(web_server_handler, "0.0.0.0", 8765):
            await asyncio.Future()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(main())


def parse(data):
    gear_raw = data[0]
    steer = data[3]
    pedal = data[4]
    ignition_raw = data[1]
    led_btn_raw = data[15] if len(data) > 15 else 0

    forward = (gear_raw == 32)
    reverse = (gear_raw == 16)
    ignition = (ignition_raw == 16)

    # 실제 매핑 전까지 참고용만 유지
    led_press = (led_btn_raw == 255)

    return steer, pedal, reverse, forward, ignition, led_press


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def calc_target_speeds(steer, pedal, gear_name):
    if pedal < 128:
        throttle = (128 - pedal) / 128.0
    else:
        throttle = 0.0

    throttle *= BASE_SPEED

    if gear_name == "REAR":
        throttle = -throttle
    elif gear_name in ("PARK", "NEUTRAL"):
        throttle = 0.0

    steer_norm = ((steer - 128) / 128.0)
    steer_norm = clamp(steer_norm, -1.0, 1.0)

    if abs(throttle) < 0.01:
        return 0.0, 0.0

    if abs(steer_norm) < STEER_DEADZONE:
        return throttle, throttle

    s = abs(steer_norm)
    if s < PIVOT_START:
        return throttle, throttle

    pivot_power = max(abs(throttle), PIVOT_POWER_MIN)
    pivot_power = clamp(pivot_power * (0.75 + s * PIVOT_POWER_GAIN), 0.0, 1.0)

    if steer_norm < 0:
        left = -pivot_power
        right = pivot_power
    else:
        left = pivot_power
        right = -pivot_power

    return left, right


def apply_output(current, target):
    if USE_INERTIA:
        return current + (target - current) * 0.35
    return target


def connect_pi():
    global pi_connected, last_error
    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        try:
            print(f"📡 {PI_IP}:{PI_PORT} 연결 시도 중...")
            sock.connect((PI_IP, PI_PORT))
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            pi_connected = True
            last_error = ""
            update_status()
            print("👍 Raspberry Pi 연결 성공")
            return sock
        except Exception as e:
            pi_connected = False
            last_error = f"Pi 연결 실패: {e}"
            update_status()
            print(f"❌ {last_error}")
            try:
                sock.close()
            except Exception:
                pass
            time.sleep(1.0)


def connect_wheel():
    global wheel_connected, last_error
    while True:
        try:
            h = hid.device()
            h.open(WHEEL_VID, WHEEL_PID)
            h.set_nonblocking(1)
            wheel_connected = True
            last_error = ""
            update_status()
            print("🎮 휠 연결 성공")
            return h
        except Exception as e:
            wheel_connected = False
            last_error = f"휠 연결 실패: {e}"
            update_status()
            print(f"❌ {last_error}")
            time.sleep(1.0)


def send_and_receive(sock, left, right, led_state):
    global pi_connected, last_error
    led_cmd = "LED_ON" if led_state else "LED_OFF"
    msg = f"{left:.2f},{right:.2f},{led_cmd}\n"
    sock.sendall(msg.encode())
    resp = sock.recv(1024).decode().strip()
    dA, dB = resp.split(",")
    pi_connected = True
    last_error = ""
    update_status()
    return float(dA), float(dB)


def pedal_display(pedal):
    if pedal < 128:
        return int(((128 - pedal) / 128.0) * 100), "A"
    if pedal > 128:
        return int(((pedal - 128) / 127.0) * 100), "B"
    return 0, "N"


threading.Thread(target=start_websocket_server, daemon=True).start()

sock = connect_pi()
h = connect_wheel()

try:
    while True:
        data = h.read(64)

        if not data:
            time.sleep(0.005)
            continue

        if len(data) < 18:
            time.sleep(0.005)
            continue

        steer, pedal, reverse, forward, ignition, led_press = parse(data)

        if ignition and not prev_ignition:
            if not engine_on:
                engine_on = True
                gear = "PARK"
                print("\n🔑 시동 ON / PARK")
            else:
                engine_on = False
                gear = "PARK"
                print("\n🔒 시동 OFF / 정지")

        if engine_on:
            if forward and not prev_forward:
                gear = "DRIVE"
                print("\n⬆️ DRIVE")
            elif reverse and not prev_reverse:
                gear = "REAR"
                print("\n⬇️ REAR")

        if ENABLE_LED_TOGGLE and led_press and not prev_led_btn:
            led_on = not led_on
            print(f"\n💡 LED {'ON' if led_on else 'OFF'}")

        prev_ignition = ignition
        prev_forward = forward
        prev_reverse = reverse
        prev_led_btn = led_press

        target_l, target_r = calc_target_speeds(steer, pedal, gear) if engine_on else (0.0, 0.0)
        current_left = apply_output(current_left, target_l)
        current_right = apply_output(current_right, target_r)

        try:
            dist_A, dist_B = send_and_receive(sock, current_left, current_right, led_on)
        except Exception as e:
            pi_connected = False
            last_error = f"Pi 송수신 오류: {e}"
            update_status()
            print(f"\n❌ {last_error}")
            try:
                sock.close()
            except Exception:
                pass
            sock = connect_pi()
            continue

        pedal_pct, pedal_type = pedal_display(pedal)
        web_data["steer"] = steer
        web_data["pedal"] = pedal_pct
        web_data["pedal_type"] = pedal_type
        update_status()

        print(
            f"\r기어:{gear} steer:{steer:3d} pedal:{pedal_pct:3d}% "
            f"L:{current_left:+.2f} R:{current_right:+.2f} "
            f"REAR(A):{dist_A:5.1f}cm FRONT(B):{dist_B:5.1f}cm",
            end=""
        )
        time.sleep(0.005)

except KeyboardInterrupt:
    print("\n시스템 종료")
finally:
    try:
        send_and_receive(sock, 0.0, 0.0, False)
    except Exception:
        pass
    try:
        sock.close()
    except Exception:
        pass
    try:
        h.close()
    except Exception:
        pass
