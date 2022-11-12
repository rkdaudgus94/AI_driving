import RPi.GPIO as IO
import time

IO.setmode(IO.BCM)

pwmPin = 14
AIN1 = 15
AIN2 = 18
encPinA = 2
encPinB = 3

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

targetDeg= 360.
ratio = 360./57./68. #168 OR 206
Kp = 1000.
Kd = 0.
Ki = 0.
dt = 0.
dt_sleep = 0.01
tolerance = 0.01

start_time = time.time()
error_prev = 0.
time_prev = 0.

while True:
    motorDeg = encoderPos * ratio

    error = targetDeg - motorDeg
    de = error-error_prev
    dt = time.time() - time_prev
    control = Kp*error + Kd*de/dt + Ki*error*dt

    error_prev = error
    time_prev = time.time()
   
    IO.output(AIN1, control >= 0)
    p.ChangeDutyCycle(min(abs(control), 15))
    
    print('P-term = %7.1f, D-term = %7.1f, I-term = %7.1f' %(Kp*error, Kd*de/dt, Ki*de*dt))
    print('time = %6.3f, enc = %d, deg = %5.1f, err = %5.1f, ctrl = %7.1f' %(time.time()-start_time, encoderPos, motorDeg, error, control))
    print('%f, %f' %(de, dt))
 
    if abs(error) <= tolerance:
        IO.output(AIN1, control >= 0)
        p.ChangeDutyCycle(0)
        break
   
    time.sleep(dt_sleep)