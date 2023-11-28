# Include the library files
import RPi.GPIO as GPIO
from time import sleep

# Include the motor control pins
ENA = 40
IN1 = 38
IN2 = 36

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(ENA,GPIO.OUT)
GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)

GPIO.output(ENA,GPIO.HIGH)
# GPIO.output(IN1,GPIO.LOW)
# GPIO.output(IN2,GPIO.LOW)


def forward():
    GPIO.output(ENA,GPIO.HIGH)
    GPIO.output(IN1,GPIO.HIGH)
    GPIO.output(IN2,GPIO.LOW)


def backward():
    GPIO.output(ENA,GPIO.HIGH)
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)

while True:
    
    #forward()
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.LOW)
    
    
GPIO.cleanup()
    
