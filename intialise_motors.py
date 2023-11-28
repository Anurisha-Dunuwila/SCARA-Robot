import RPi.GPIO as GPIO
import time
from array import *

# Define the GPIO pins for each motor
motor2_direction_pin = 11  # Motor 2 direction pin
motor2_pulse_pin = 7  # Motor 2 pulse pin

motor1_direction_pin = 16  # Motor 1 direction pin
motor1_pulse_pin = 12  # Motor 1 pulse pin

motor3_direction_pin = 13  # Motor 1 direction pin
motor3_pulse_pin = 15  # Motor 1 pulse pin

lim3_pin=35
lim2_pin=37


GPIO.setwarnings(False)
# Set the GPIO mode and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(motor2_direction_pin, GPIO.OUT)
GPIO.setup(motor2_pulse_pin, GPIO.OUT)

GPIO.setup(motor1_direction_pin, GPIO.OUT)
GPIO.setup(motor1_pulse_pin, GPIO.OUT)

GPIO.setup(motor3_direction_pin, GPIO.OUT)
GPIO.setup(motor3_pulse_pin, GPIO.OUT)

GPIO.setup(lim3_pin, GPIO.IN,  pull_up_down=GPIO.PUD_UP)
GPIO.setup(lim2_pin, GPIO.IN,  pull_up_down=GPIO.PUD_UP)


# Function to rotate the motor by a specified number of steps
def rotate_motor(direction_pin, pulse_pin, steps):
    # Set the direction pin based on the desired direction
    GPIO.output(direction_pin, GPIO.HIGH if steps > 0 else GPIO.LOW)

    # Generate pulses to rotate the motor
    for _ in range(abs(steps)):
        GPIO.output(pulse_pin, GPIO.HIGH)
        time.sleep(0.001)  # Adjust delay as needed
        GPIO.output(pulse_pin, GPIO.LOW)
        time.sleep(0.001)  # Adjust delay as needed

# Rotate both motors to a known angle (in degrees)
def rotate_to_angle2(direction_pin, pulse_pin, angle):
    steps = int((2*angle / 360) * 3200)  # Adjust the number of steps as needed
    rotate_motor(direction_pin, pulse_pin, steps)
    
def rotate_to_angle1(direction_pin, pulse_pin, angle):
    steps = int((2.5*angle / 360) * 3200)  # Adjust the number of steps as needed
    rotate_motor(direction_pin, pulse_pin, steps)
    
def rotate_to_angle3(direction_pin, pulse_pin, angle):
    steps = int((2*angle / 360) * 800)  # Adjust the number of steps as needed
    rotate_motor(direction_pin, pulse_pin, steps)
    
while True:
    rotate_motor(motor3_direction_pin, motor3_pulse_pin, -1)
    if(GPIO.input(lim3_pin)==0):
           print("arrived_3")
           break
        
while True:
    rotate_motor(motor2_direction_pin, motor2_pulse_pin, -1)
    if(GPIO.input(lim2_pin)==0):
           print("arrived_2")
           break
        

