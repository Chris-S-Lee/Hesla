import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# L298N #1 (왼쪽 앞, 왼쪽 뒤)
ENA1 = 18
IN1 = 23
IN2 = 24

ENB1 = 19
IN3 = 27
IN4 = 22

# L298N #2 (오른쪽 앞, 오른쪽 뒤)
ENA2 = 12
IN5 = 5
IN6 = 6

ENB2 = 13
IN7 = 16
IN8 = 26

pins = [ENA1, IN1, IN2, ENB1, IN3, IN4, ENA2, IN5, IN6, ENB2, IN7, IN8]

for pin in pins:
GPIO.setup(pin, GPIO.OUT)

# PWM 설정
pwm_LF = GPIO.PWM(ENA1, 1000) # 왼쪽 앞
pwm_LR = GPIO.PWM(ENB1, 1000) # 왼쪽 뒤
pwm_RF = GPIO.PWM(ENA2, 1000) # 오른쪽 앞
pwm_RR = GPIO.PWM(ENB2, 1000) # 오른쪽 뒤

pwm_LF.start(0)
pwm_LR.start(0)
pwm_RF.start(0)
pwm_RR.start(0)

def set_motor(in_a, in_b, speed, forward=True):
if forward:
GPIO.output(in_a, GPIO.HIGH)
GPIO.output(in_b, GPIO.LOW)
else:
GPIO.output(in_a, GPIO.LOW)
GPIO.output(in_b, GPIO.HIGH)

def stop_motor(in_a, in_b):
GPIO.output(in_a, GPIO.LOW)
GPIO.output(in_b, GPIO.LOW)

def forward(speed=70):
set_motor(IN1, IN2, speed, True) # 왼쪽 앞
set_motor(IN3, IN4, speed, True) # 왼쪽 뒤
set_motor(IN5, IN6, speed, True) # 오른쪽 앞
set_motor(IN7, IN8, speed, True) # 오른쪽 뒤

pwm_LF.ChangeDutyCycle(speed)
pwm_LR.ChangeDutyCycle(speed)
pwm_RF.ChangeDutyCycle(speed)
pwm_RR.ChangeDutyCycle(speed)

def backward(speed=70):
set_motor(IN1, IN2, speed, False)
set_motor(IN3, IN4, speed, False)
set_motor(IN5, IN6, speed, False)
set_motor(IN7, IN8, speed, False)

pwm_LF.ChangeDutyCycle(speed)
pwm_LR.ChangeDutyCycle(speed)
pwm_RF.ChangeDutyCycle(speed)
pwm_RR.ChangeDutyCycle(speed)

def left_turn(speed=70):
# 왼쪽 정지, 오른쪽 전진
stop_motor(IN1, IN2)
stop_motor(IN3, IN4)
pwm_LF.ChangeDutyCycle(0)
pwm_LR.ChangeDutyCycle(0)

set_motor(IN5, IN6, speed, True)
set_motor(IN7, IN8, speed, True)
pwm_RF.ChangeDutyCycle(speed)
pwm_RR.ChangeDutyCycle(speed)

def right_turn(speed=70):
# 오른쪽 정지, 왼쪽 전진
set_motor(IN1, IN2, speed, True)
set_motor(IN3, IN4, speed, True)
pwm_LF.ChangeDutyCycle(speed)
pwm_LR.ChangeDutyCycle(speed)

stop_motor(IN5, IN6)
stop_motor(IN7, IN8)
pwm_RF.ChangeDutyCycle(0)
pwm_RR.ChangeDutyCycle(0)

def stop_all():
stop_motor(IN1, IN2)
stop_motor(IN3, IN4)
stop_motor(IN5, IN6)
stop_motor(IN7, IN8)

pwm_LF.ChangeDutyCycle(0)
pwm_LR.ChangeDutyCycle(0)
pwm_RF.ChangeDutyCycle(0)
pwm_RR.ChangeDutyCycle(0)

try:
forward(70)
time.sleep(3)

stop_all()
time.sleep(1)

backward(70)
time.sleep(3)

stop_all()
time.sleep(1)

left_turn(70)
time.sleep(2)

stop_all()
time.sleep(1)

right_turn(70)
time.sleep(2)

stop_all()

finally:
pwm_LF.stop()
pwm_LR.stop()
pwm_RF.stop()
pwm_RR.stop()
GPIO.cleanup()



