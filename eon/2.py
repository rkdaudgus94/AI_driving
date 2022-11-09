# 엔코더 읽는 코드

import RPi.GPIO as IO
import time

pwmPin = 19
AIN1 = 5
AIN2 = 6
encPinA = 23
encPinB = 24

IO.setmode(IO.BCM)
IO.setwarnings(False)
IO.setup(encPinA, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(encPinB, IO.IN, pull_up_down=IO.PUD_UP)
IO.setup(pwmPin,IO.OUT, initial=IO.LOW)
IO.setup(AIN1,IO.OUT, initial=IO.LOW)
IO.setup(AIN2,IO.OUT, initial=IO.LOW)

p = IO.PWM(13, 100)
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

while True:
    IO.output(AIN1, IO.HIGH)
    p.ChangeDutyCycle(100)

    print('PinA : %d, encoder : %d' %(encPinA, encoderPos))
    print('PinB : %d, encoder : %d' %(encPinB, encoderPos))
    
    time.sleep(0.5)