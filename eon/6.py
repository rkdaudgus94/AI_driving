# 모터 pwm

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

AIN1= 15
AIN2= 18
PWMA= 14

#각 핀을 출력 핀으로 설정 
GPIO.setup(AIN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(AIN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(PWMA, GPIO.OUT, initial=GPIO.LOW)

#PWM 객체 인스턴스 작성(출력 핀:14, 주파수 100)
p = GPIO.PWM(PWMA, 100)
p.start(0)

try :
    while True :  
        GPIO.output(AIN1, GPIO.HIGH)
        GPIO.output(AIN2, GPIO.LOW)
        p.ChangeDutyCycle(100)

        print('회전')

        time.sleep(0.5)

except KeyboardInterrupt: 
    pass 

p.stop() 

GPIO.cleanup()