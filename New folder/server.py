import socket
import time
import threading
import atexit

import cv2
from flask import Flask, Response
import RPi.GPIO as GPIO

# =========================
# 핀맵
# =========================
# L298N A = 오른쪽
A_IN1 = 23
A_IN2 = 24
A_IN3 = 27
A_IN4 = 22
A_ENA = 18
A_ENB = 19

# L298N B = 왼쪽
B_IN1 = 5
B_IN2 = 6
B_IN3 = 16
B_IN4 = 26
B_ENA = 12
B_ENB = 13

# LED 릴레이
LED_RELAY = 21

# 초음파 센서 A = 후방
TRIG_A = 10
ECHO_A = 9

# 초음파 센서 B = 전방
TRIG_B = 11
ECHO_B = 25

# =========================
# 네트워크 / 카메라 설정
# =========================
HOST = "0.0.0.0"
PORT = 9999

CAM_HOST = "0.0.0.0"
CAM_PORT = 8080
FRONT_CAM_INDEX = 0   # /dev/video0
REAR_CAM_INDEX = 2    # /dev/video2
CAM_WIDTH = 640
CAM_HEIGHT = 360
CAM_FPS = 20

PWM_FREQ = 1000
MAX_MOTOR_ABS = 1.00
ULTRA_TIMEOUT = 0.03
STOP_DISTANCE_CM = 50.0

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in [
    A_IN1, A_IN2, A_IN3, A_IN4, A_ENA, A_ENB,
    B_IN1, B_IN2, B_IN3, B_IN4, B_ENA, B_ENB,
    LED_RELAY, TRIG_A, TRIG_B
]:
    GPIO.setup(pin, GPIO.OUT)

for pin in [ECHO_A, ECHO_B]:
    GPIO.setup(pin, GPIO.IN)

GPIO.output(TRIG_A, False)
GPIO.output(TRIG_B, False)
GPIO.output(LED_RELAY, False)

pwm_a_ena = GPIO.PWM(A_ENA, PWM_FREQ)
pwm_a_enb = GPIO.PWM(A_ENB, PWM_FREQ)
pwm_b_ena = GPIO.PWM(B_ENA, PWM_FREQ)
pwm_b_enb = GPIO.PWM(B_ENB, PWM_FREQ)

for pwm in [pwm_a_ena, pwm_a_enb, pwm_b_ena, pwm_b_enb]:
    pwm.start(0)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def stop_all():
    for pin in [A_IN1, A_IN2, A_IN3, A_IN4, B_IN1, B_IN2, B_IN3, B_IN4]:
        GPIO.output(pin, False)
    for pwm in [pwm_a_ena, pwm_a_enb, pwm_b_ena, pwm_b_enb]:
        pwm.ChangeDutyCycle(0)


def set_dual_hbridge(in1, in2, in3, in4, pwm1, pwm2, speed):
    speed = clamp(speed, -MAX_MOTOR_ABS, MAX_MOTOR_ABS)
    duty = abs(speed) * 100.0

    if abs(speed) < 0.01:
        GPIO.output(in1, False)
        GPIO.output(in2, False)
        GPIO.output(in3, False)
        GPIO.output(in4, False)
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(0)
        return

    if speed > 0:
        GPIO.output(in1, True)
        GPIO.output(in2, False)
        GPIO.output(in3, True)
        GPIO.output(in4, False)
    else:
        GPIO.output(in1, False)
        GPIO.output(in2, True)
        GPIO.output(in3, False)
        GPIO.output(in4, True)

    pwm1.ChangeDutyCycle(duty)
    pwm2.ChangeDutyCycle(duty)


def set_right_motor(speed):
    set_dual_hbridge(A_IN1, A_IN2, A_IN3, A_IN4, pwm_a_ena, pwm_a_enb, speed)


def set_left_motor(speed):
    set_dual_hbridge(B_IN1, B_IN2, B_IN3, B_IN4, pwm_b_ena, pwm_b_enb, speed)


def set_led(onoff):
    GPIO.output(LED_RELAY, bool(onoff))


def read_ultrasonic(trig, echo):
    GPIO.output(trig, False)
    time.sleep(0.000002)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_wait = time.time()
    pulse_start = start_wait
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        if pulse_start - start_wait > ULTRA_TIMEOUT:
            return 999.0

    start_high = time.time()
    pulse_end = start_high
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if pulse_end - start_high > ULTRA_TIMEOUT:
            return 999.0

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    if distance <= 0 or distance > 999:
        return 999.0
    return round(distance, 1)


def safe_distance_logic(left_cmd, right_cmd, rear_dist_a, front_dist_b):
    moving_forward = (left_cmd > 0.01) or (right_cmd > 0.01)
    moving_backward = (left_cmd < -0.01) or (right_cmd < -0.01)

    if moving_forward and front_dist_b < STOP_DISTANCE_CM:
        return 0.0, 0.0

    if moving_backward and rear_dist_a < STOP_DISTANCE_CM:
        return 0.0, 0.0

    return left_cmd, right_cmd


# =========================
# 카메라 스트리밍
# =========================
class CameraStream:
    def __init__(self, device_index, name, width=640, height=360, fps=20):
        self.device_index = device_index
        self.name = name
        self.width = width
        self.height = height
        self.fps = fps

        self.cap = None
        self.frame = None
        self.lock = threading.Lock()
        self.running = True

        self.open_camera()
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def open_camera(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass

        self.cap = cv2.VideoCapture(self.device_index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        print(f"🎥 {self.name} 카메라 열기 시도: /dev/video{self.device_index}")

    def update(self):
        fail_count = 0
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                fail_count += 1
                time.sleep(1.0)
                self.open_camera()
                continue

            ok, frame = self.cap.read()
            if ok and frame is not None:
                fail_count = 0
                with self.lock:
                    self.frame = frame
            else:
                fail_count += 1
                time.sleep(0.05)
                if fail_count >= 20:
                    print(f"⚠️ {self.name} 프레임 읽기 실패, 재연결 시도")
                    self.open_camera()
                    fail_count = 0

    def get_jpeg(self):
        with self.lock:
            if self.frame is None:
                return None
            ok, buffer = cv2.imencode(".jpg", self.frame)
            if not ok:
                return None
            return buffer.tobytes()

    def release(self):
        self.running = False
        time.sleep(0.2)
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass


front_cam = CameraStream(FRONT_CAM_INDEX, "전방", CAM_WIDTH, CAM_HEIGHT, CAM_FPS)
rear_cam = CameraStream(REAR_CAM_INDEX, "후방", CAM_WIDTH, CAM_HEIGHT, CAM_FPS)

app = Flask(__name__)


def mjpeg_generator(cam):
    while True:
        jpg = cam.get_jpeg()
        if jpg is None:
            time.sleep(0.03)
            continue
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        )


@app.route("/")
def camera_root():
    return "Hesla camera server running"


@app.route("/front.mjpg")
def front_feed():
    return Response(
        mjpeg_generator(front_cam),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/rear.mjpg")
def rear_feed():
    return Response(
        mjpeg_generator(rear_cam),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


def run_camera_server():
    print(f"📷 카메라 스트리밍 서버 시작: {CAM_HOST}:{CAM_PORT}")
    print(f"   전방: http://<PI_IP>:{CAM_PORT}/front.mjpg")
    print(f"   후방: http://<PI_IP>:{CAM_PORT}/rear.mjpg")
    app.run(host=CAM_HOST, port=CAM_PORT, threaded=True, use_reloader=False)


# =========================
# 주행 TCP 서버
# =========================
def handle_client(conn, addr):
    print(f"✅ 클라이언트 연결: {addr}")
    buffer = ""
    conn.settimeout(2.0)

    try:
        while True:
            try:
                data = conn.recv(1024)
            except socket.timeout:
                continue
            except (ConnectionResetError, BrokenPipeError):
                print(f"❌ 클라이언트 연결 리셋: {addr}")
                break

            if not data:
                print(f"❌ 클라이언트 연결 종료: {addr}")
                break

            buffer += data.decode(errors="ignore")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue

                try:
                    left_s, right_s, led_cmd = line.split(",")
                    left_cmd = float(left_s)
                    right_cmd = float(right_s)
                    led_on = (led_cmd == "LED_ON")

                    dist_a = read_ultrasonic(TRIG_A, ECHO_A)  # rear
                    dist_b = read_ultrasonic(TRIG_B, ECHO_B)  # front

                    left_cmd, right_cmd = safe_distance_logic(left_cmd, right_cmd, dist_a, dist_b)

                    set_left_motor(left_cmd)
                    set_right_motor(right_cmd)
                    set_led(led_on)

                    resp = f"{dist_a:.1f},{dist_b:.1f}\n"
                    conn.sendall(resp.encode())

                    print(
                        f"L={left_cmd:+.2f} R={right_cmd:+.2f} LED={'ON' if led_on else 'OFF'} "
                        f"| REAR(A)={dist_a:.1f}cm FRONT(B)={dist_b:.1f}cm"
                    )

                except Exception as e:
                    print(f"⚠️ 명령 처리 오류: {e} / raw={line}")
                    stop_all()
                    try:
                        conn.sendall(b"999.0,999.0\n")
                    except Exception:
                        break

    finally:
        stop_all()
        set_led(False)
        try:
            conn.close()
        except Exception:
            pass


def cleanup():
    print("🧹 정리 중...")
    stop_all()
    set_led(False)

    for pwm in [pwm_a_ena, pwm_a_enb, pwm_b_ena, pwm_b_enb]:
        try:
            pwm.stop()
        except Exception:
            pass

    try:
        front_cam.release()
    except Exception:
        pass

    try:
        rear_cam.release()
    except Exception:
        pass

    try:
        GPIO.cleanup()
    except Exception:
        pass


atexit.register(cleanup)


def main():
    stop_all()
    set_led(False)

    cam_thread = threading.Thread(target=run_camera_server, daemon=True)
    cam_thread.start()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    print(f"🚗 Raspberry Pi 주행 서버 시작: {HOST}:{PORT}")
    print(f"⚙️ 최대 출력={MAX_MOTOR_ABS:.2f}, 차단거리={STOP_DISTANCE_CM:.1f}cm")
    print("   전진 차단 = 전방(B), 후진 차단 = 후방(A)")
    print("대기 중...")

    try:
        while True:
            conn, addr = server.accept()
            handle_client(conn, addr)
    except KeyboardInterrupt:
        print("\n서버 종료")
    finally:
        try:
            server.close()
        except Exception:
            pass
        cleanup()


if __name__ == "__main__":
    main()
