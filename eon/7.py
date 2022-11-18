import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO_RP = 15
GPIO_RN = 18
GPIO_EN = 14

GPIO.setup(GPIO_RP, GPIO.OUT) # IN1
GPIO.setup(GPIO_RN, GPIO.OUT) # IN2
GPIO.setup(GPIO_EN, GPIO.OUT) # PWM

##단계(1~10)를 넣어 모터의 속도를 조절
def setSpeed(speed,p):
    p.ChangeDutyCycle(speed*10)
      
try:
    p = GPIO.PWM(GPIO_EN, 100) # 100hz
    p.start(0) # start the PWM on 0% duty cycle  
      
    while True:
        for i in range(10):
             GPIO.output(GPIO_RP, True)
             GPIO.output(GPIO_RN, False)
             setSpeed(i, p)            
             time.sleep(1)
finally:    
    GPIO.cleanup()