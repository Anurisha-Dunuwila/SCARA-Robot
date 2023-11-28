import RPi.GPIO as GPIO
import time

lim1_pin=35
lim2_pin=37

GPIO.setwarnings(False)
# Set the GPIO mode and pins
GPIO.setmode(GPIO.BOARD)



GPIO.setup(lim1_pin, GPIO.IN,  pull_up_down=GPIO.PUD_UP)
GPIO.setup(lim2_pin, GPIO.IN,  pull_up_down=GPIO.PUD_UP)
while True:
    if((GPIO.input(lim1_pin)==1)and (GPIO.input(lim2_pin)==0)):
       print("arrived_1")
    elif((GPIO.input(lim2_pin)==1) and (GPIO.input(lim1_pin)==0)):
       print("arrived_2")
    elif((GPIO.input(lim2_pin)==1) and (GPIO.input(lim1_pin)==1)):
       print("arrived_3")
    else:
       print("no")
