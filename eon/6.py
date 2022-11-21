import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO_TRIGGER = 2
GPIO_ECHO = 3
print("start")

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

try:
    while True:
        StartTime = time.time()
        StopTime = time.time()
        GPIO.output(GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)

        while GPIO.input(GPIO_ECHO) == 0:
            StartTime = time.time()
        
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()
        
        TimeElapsed = StopTime - StartTime
        distance = round((TimeElapsed * 34300) / 2, 2)
        print("Distance = ", distance, "cm")
        time.sleep(1)

except KeyboardInterrupt:
    pass