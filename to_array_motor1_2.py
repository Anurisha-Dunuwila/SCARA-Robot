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


GPIO.setwarnings(False)
# Set the GPIO mode and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(motor2_direction_pin, GPIO.OUT)
GPIO.setup(motor2_pulse_pin, GPIO.OUT)

GPIO.setup(motor1_direction_pin, GPIO.OUT)
GPIO.setup(motor1_pulse_pin, GPIO.OUT)

GPIO.setup(motor3_direction_pin, GPIO.OUT)
GPIO.setup(motor3_pulse_pin, GPIO.OUT)


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


req_array=[[50,50,100],[50,50,100],[-90,-90,-100]]
array_length = len(req_array)
# print(array_length)

for r in range (array_length):
#     print(req_array[r][0])
    rotate_to_angle1(motor1_direction_pin, motor1_pulse_pin, req_array[r][0])
    rotate_to_angle2(motor2_direction_pin, motor2_pulse_pin, req_array[r][1])
    rotate_to_angle3(motor3_direction_pin, motor3_pulse_pin, req_array[r][2])
    time. sleep(1)

# Clean up GPIO pins
GPIO.cleanup()

