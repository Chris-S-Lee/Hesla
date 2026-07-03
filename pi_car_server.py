import socket
import time
import math
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

# 초음파 센서 A
TRIG_A = 10
ECHO_A = 9

# 초음파 센서 B
TRIG_B = 11
ECHO_B = 25

HOST = "0.0.0.0"
PORT = 9999

PWM_FREQ = 1000

# 안전 제한
MAX_MOTOR_ABS = 0.85
ULTRA_TIMEOUT = 0.03   # 30ms
STOP_DISTANCE_CM = 20.0

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

MOTOR_PINS = [
    A_IN1, A_IN2, A_IN3, A_IN4, A_ENA, A_ENB,
    B_IN1, B_IN2, B_IN3, B_IN4, B_ENA, B_ENB,
    LED_RELAY, TRIG_A, ECHO_A, TRIG_B, ECHO_B
]

for pin in [A_IN1, A_IN2, A_IN3, A_IN4, A_ENA, A_ENB,
            B_IN1, B_IN2, B_IN3, B_IN4, B_ENA, B_ENB,
            LED_RELAY, TRIG_A, TRIG_B]:
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
        # 정방향
        GPIO.output(in1, True)
        GPIO.output(in2, False)
        GPIO.output(in3, True)
        GPIO.output(in4, False)
    else:
        # 역방향
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
    # 트리거 펄스
    GPIO.output(trig, False)
    time.sleep(0.000002)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_wait = time.time()
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        if pulse_start - start_wait > ULTRA_TIMEOUT:
            return 999.0

    start_high = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if pulse_end - start_high > ULTRA_TIMEOUT:
            return 999.0

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    if distance <= 0 or distance > 999:
        return 999.0
    return round(distance, 1)


def safe_distance_logic(left_cmd, right_cmd, dist_a, dist_b):
    # 전진 중 가까우면 정지
    if left_cmd > 0 or right_cmd > 0:
        if dist_a < STOP_DISTANCE_CM or dist_b < STOP_DISTANCE_CM:
            return 0.0, 0.0
    return left_cmd, right_cmd


def handle_client(conn, addr):
    print(f"✅ 클라이언트 연결: {addr}")
    buffer = ""

    try:
        while True:
            data = conn.recv(1024)
            if not data:
                print("❌ 클라이언트 연결 종료")
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

                    dist_a = read_ultrasonic(TRIG_A, ECHO_A)
                    dist_b = read_ultrasonic(TRIG_B, ECHO_B)

                    left_cmd, right_cmd = safe_distance_logic(left_cmd, right_cmd, dist_a, dist_b)

                    set_left_motor(left_cmd)
                    set_right_motor(right_cmd)
                    set_led(led_on)

                    resp = f"{dist_a:.1f},{dist_b:.1f}\n"
                    conn.sendall(resp.encode())

                    print(
                        f"L={left_cmd:+.2f} R={right_cmd:+.2f} LED={'ON' if led_on else 'OFF'} "
                        f"| A={dist_a:.1f}cm B={dist_b:.1f}cm"
                    )

                except Exception as e:
                    print(f"⚠️ 명령 처리 오류: {e} / raw={line}")
                    stop_all()
                    try:
                        conn.sendall(b"999.0,999.0\n")
                    except Exception:
                        pass

    finally:
        stop_all()
        try:
            conn.close()
        except Exception:
            pass


def main():
    stop_all()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    print(f"🚗 Raspberry Pi 주행 서버 시작: {HOST}:{PORT}")
    print("대기 중...")

    try:
        while True:
            conn, addr = server.accept()
            handle_client(conn, addr)
    except KeyboardInterrupt:
        print("\n서버 종료")
    finally:
        stop_all()
        set_led(False)
        for pwm in [pwm_a_ena, pwm_a_enb, pwm_b_ena, pwm_b_enb]:
            pwm.stop()
        server.close()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
