# 엔코더를 이용해 DC 모터 각도 조절

import RPi.GPIO as IO
import time

pwmPin = 14 # 모터드라이버 ENA
AIN1 = 15 # IN 1
AIN2 = 18 # IN 2
encPinA = 2 # 보라색 (A)
encPinB = 3 # 파랑색 (B)

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinB, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPin, IO.OUT, initial=IO.LOW)
IO.setup(AIN1, IO.OUT, initial=IO.LOW)
IO.setup(AIN2, IO.OUT, initial=IO.LOW)

p = IO.PWM(14, 100)
p.start(0)

encoderPos = 0
setha = 0

# 엔코더 인터럽트 제어 함수
def encoderA(encPinA):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos += 1 
    else:
        encoderPos -= 1
   
def encoderB(encPinB):
    global encoderPos
    if IO.input(encPinA) == IO.input(encPinB):
        encoderPos -= 1
    else:
        encoderPos += 1

IO.add_event_detect(encPinA, IO.BOTH, callback=encoderA)
IO.add_event_detect(encPinB, IO.BOTH, callback=encoderB)

# PID 제어
ratio = 360./90./52. # 한 바퀴에 약 4100펄스

# PID 상수
kp = 0. 
kd = 0.
ki = 0.

dt = 0.
dt_sleep = 0.01

start_time =time.time()
error_prev = 0.
time_prev = 0.

try:
    while True:
        # motorDeg : 실제 모터 각도
        motorDeg = encoderPos * ratio
        # error : 원하는 각도 - 실제 모터 각도
        error = setha - motorDeg
        de = error - error_prev
        dt = time.time() - time_prev
    
        control = (kp * error) + (kd * de/dt) + (ki * error * dt)

        error_prev = error
        time_prev = time.time()

        IO.output(AIN1, IO.HIGH)
        IO.output(AIN2, IO.LOW)
                
        p.ChangeDutyCycle(20)

        # RESET(현재 위치를 기준으로 각도를 읽어야 하므로 각도를 입력하기 전에 RESET을 해줘야 함.)
        if (360 <= motorDeg) :
            IO.output(AIN1, IO.LOW)
            IO.output(AIN2, IO.LOW)
            p.ChangeDutyCycle(0)

        print('enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' %(encoderPos, motorDeg, error, control))
        print('P-term = %7.1f' %(kp*error))

        time.sleep(dt_sleep)

# Crtl + c 누르면 모터 작동 멈춤
except KeyboardInterrupt: 
    pass 

p.stop() 

IO.cleanup()