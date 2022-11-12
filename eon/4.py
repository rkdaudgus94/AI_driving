# 엔코더를 이용해 DC 모터 각도 조절

import RPi.GPIO as IO
import time

pwmPin = 14
AIN1 = 15
AIN2 = 18
encPinA = 2 # 보라색 (A)
encPinB = 3 # 파랑색 (B)

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinB, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPin,IO.OUT, initial=IO.LOW)
IO.setup(AIN1,IO.OUT, initial=IO.LOW)
IO.setup(AIN2,IO.OUT, initial=IO.LOW)

p = IO.PWM(14, 100)
p.start(0)

encoderPos = 0

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

setha = 0

kp = 50.
kd = 0.
ki = 0.
dt = 0.
dt_sleep = 0.01
tolerance = 0.01

start_time = time.time()
error_prev = 0.
time_prev = 0.

try:
    setha = int(input('각도를 입력하시오 : '))

    while True:
        motorDeg = encoderPos * ratio
        error = setha - motorDeg
    
        de = error - error_prev
        dt = time.time() - time_prev
        control = (kp*error) + (kd*de/dt) + (ki*error*dt)

        error_prev = error
        time_prev = time.time()
        
        #RESET
        if (setha == 00) :
            IO.output(AIN1, IO.LOW)
            IO.output(AIN2, IO.LOW)
            p.ChangeDutyCycle(0)

            encoderPos = 0
            setha = 0
            motorDeg = 0
            error = 0

        print('RESET')

        if(setha < 0) :
            IO.output(AIN1, IO.HIGH)
            IO.output(AIN2, IO.LOW)

            if ((setha >= motorDeg) & (control >= 0)) :
                IO.output(AIN1, IO.LOW)
                IO.output(AIN2, IO.LOW)
                p.ChangeDutyCycle(0)

            p.ChangeDutyCycle(min(abs(control), 100))


        elif (setha > 0) :
            IO.output(AIN1, IO.LOW)
            IO.output(AIN2, IO.HIGH)

            if((setha <= motorDeg) & (control <= 0)) :
                IO.output(AIN1, IO.LOW)
                IO.output(AIN2, IO.LOW)
                p.ChangeDutyCycle(0)

            p.ChangeDutyCycle(min(abs(control), 100))

        print('setha = %d' %(setha))
        print('P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' %(kp*error, kd*de/dt, ki*de*dt))
        print('enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' %(encoderPos, motorDeg, error, control))
        print('%f, %f' %(de, dt))
    
        time.sleep(0.5)

        

except KeyboardInterrupt: 
    pass 

p.stop() 

IO.cleanup()