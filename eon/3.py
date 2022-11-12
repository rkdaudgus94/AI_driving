# 360도 회전하기 

import RPi.GPIO as IO
import time

pwmPin = 14
AIN1 = 15
AIN2 = 18
encPinA = 2
encPinB = 3

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

targetDeg = 360.
ratio = 360./90./41.
kp = 10.
kd = 0.
ki = 0.
dt = 0.
dt_sleep = 0.01
tolerance = 0.01

start_time = time.time()
error_prev = 0.
time_prev = 0.


while True:
    motorDeg = encoderPos * ratio

    error = targetDeg - motorDeg
    de = error - error_prev
    dt = time.time() - time_prev
    control = (kp*error) + (kd*de/dt) + (ki*error*dt)

    error_prev = error
    time_prev = time.time()

    IO.output(AIN1, control >= 0)
    # IO.output(AIN2, IO.LOW)
    p.ChangeDutyCycle(min(abs(control), 100))

    print('P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' %(kp*error, kd*de/dt, ki*de*dt))
    print('time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' %(time.time()-start_time, encoderPos, motorDeg, error, control))
    print('%f, %f' %(de, dt))
    
    if abs(error) <= tolerance :
        IO.ouput(AIN1, control >= 0)
        IO.ouput(AIN2, control >= 0)
        p.ChangeDutyCycle(100)
        break
    
    # time.sleep(0.5)
    time.sleep(dt_sleep)