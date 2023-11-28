import RPi.GPIO as GPIO
import time

# Define the GPIO pins for each motor
motor1_direction_pin = 11  # Motor 1 direction pin
motor1_pulse_pin = 7  # Motor 1 pulse pin


GPIO.setwarnings(False)
# Set the GPIO mode and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(motor1_direction_pin, GPIO.OUT)
GPIO.setup(motor1_pulse_pin, GPIO.OUT)

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

# Rotate both motors to 0-degree position
#rotate_motor(motor1_direction_pin, motor1_pulse_pin, -800)

# Rotate both motors to a known angle (in degrees)
def rotate_to_angle(direction_pin, pulse_pin, angle):
    steps = int((2*angle / 360) * 3200)  # Adjust the number of steps as needed
    rotate_motor(direction_pin, pulse_pin, steps)

# Rotate both motors to a known angle
angle =-180  # Specify the desired angle
rotate_to_angle(motor1_direction_pin, motor1_pulse_pin, angle)

# Clean up GPIO pins
GPIO.cleanup()
