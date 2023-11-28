
from threading import Thread
import customtkinter
from tkdial import Meter
from datetime import datetime
import pandas as pd

from pandastable import Table

import RPi.GPIO as GPIO
import time
from array import *

import cv2
from yolov4_exec import exec_get_traverse_path

import pyrebase

config ={
    "apiKey": "AIzaSyAXsca8gJErV9fwJqWT8JZpMbzpAmNDsK4",
    "authDomain": "scararobotapp.firebaseapp.com",
    "databaseURL": "https://scararobotapp-default-rtdb.asia-southeast1.firebasedatabase.app/",
    "storageBucket": "scararobotapp.appspot.com"
    }

firebase=pyrebase.initialize_app(config)
db=firebase.database()
db.child('fk_values').child("run").set(0)
run=db.child('fk_values').child("run").get()
print("Initial RUN:",run.val())

DEVICE="PC" #or "APP"

auto_m_thickness=0
auto_m_type=""



# Define the GPIO pins for each motor
motor2_direction_pin = 11  # Motor 2 direction pin
motor2_pulse_pin = 7  # Motor 2 pulse pin

motor1_direction_pin = 16  # Motor 1 direction pin
motor1_pulse_pin = 12  # Motor 1 pulse pin

motor3_direction_pin = 13  # Motor 1 direction pin
motor3_pulse_pin = 15  # Motor 1 pulse pin

lim3_pin=33
lim2_pin=35
lim1_pin=37


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
GPIO.setup(lim1_pin, GPIO.IN,  pull_up_down=GPIO.PUD_UP)



# -----------------------GUI-------------------------------

#Function for btn_stop_w button
def closeW():
    window.quit()
    window.destroy()
    
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
    
def traverse(path):
    #dummy_path=[(30, 20, 200), (20,30,300), (60, 40, -400 )]
    for point in path:
        theta1 = point[0]
        theta2 = point[1]
        Z = point[2]

        print("(θ1=", theta1, "θ2=", theta2, "Z=", Z,")")
        
        rotate_to_angle1(motor1_direction_pin, motor1_pulse_pin, int(theta1))
        rotate_to_angle2(motor2_direction_pin, motor2_pulse_pin, int(theta2))
        rotate_to_angle3(motor3_direction_pin, motor3_pulse_pin, int(Z))
        time.sleep(1)
        rotate_to_angle3(motor3_direction_pin, motor3_pulse_pin, 500)                
        time.sleep(3)
        rotate_to_angle3(motor3_direction_pin, motor3_pulse_pin, -500)
        time.sleep(1)
        
#     for dummy_point in dummy_path:
#         
#         thresholdVal1 = dummy_point[0]
#         thresholdVal2 = dummy_point[1]
#         thresholdVal3 = dummy_point[2]
# 
#         rotate_to_angle1(motor1_direction_pin, motor1_pulse_pin, int(thresholdVal1))
#         rotate_to_angle2(motor2_direction_pin, motor2_pulse_pin, int(thresholdVal2))
#         rotate_to_angle3(motor3_direction_pin, motor3_pulse_pin, int(thresholdVal3))


def auto():
    if DEVICE=="PC":
        global thresholdVal
        global screw_op

        thresholdVal = entry.get()
        screw_op = optionmenu.get()

    #     camera = cv2.VideoCapture(0)
    #     time.sleep(2)
    #     return_value, image = camera.read()
    #     cv2.imshow("pre", image)
    #     cv2.waitKey(300)
    # #     cv2.imwrite('opencv.png', image)
    #     camera.release()
    #     cv2.destroyAllWindows()

        #init_pos()

        # traverse the path in the shortest way
        path = exec_get_traverse_path(screw_op, thresholdVal)
        traverse(path)

        # rotate_to_angle3(motor3_direction_pin, motor3_pulse_pin, int(thresholdVal))
        print(screw_op)
        
def manual():
    if DEVICE=="PC":
        global thresholdVal1 
        global thresholdVal2
        global thresholdVal3
        
        thresholdVal1 = entry1.get()
        thresholdVal2 = entry2.get()
        thresholdVal3 = entry3.get()
        
        #init_pos()
        
        rotate_to_angle1(motor1_direction_pin, motor1_pulse_pin, int(thresholdVal1))
        rotate_to_angle2(motor2_direction_pin, motor2_pulse_pin, int(thresholdVal2))
        rotate_to_angle3(motor3_direction_pin, motor3_pulse_pin, int(thresholdVal3))
        
def init_pos():
    
    while True:
        rotate_motor(motor1_direction_pin, motor1_pulse_pin, -1)
        if(GPIO.input(lim1_pin)==0):
           print("arrived_1")
           break
        
    while True:
        rotate_motor(motor2_direction_pin, motor2_pulse_pin, -1)
        if(GPIO.input(lim2_pin)==0):
           print("arrived_2")
           break
        
    while True:
        rotate_motor(motor3_direction_pin, motor3_pulse_pin, -1)
        if(GPIO.input(lim3_pin)==0):
           print("arrived_3")
           break
        

def toggle_device():
    global DEVICE
    if DEVICE=="PC":
        DEVICE="APP"
        db.child("fk_values").update({"device":"APP"})

    else:
        DEVICE="PC"
        db.child("fk_values").update({"device":"PC"})

def mode_handler(event):
    
    print(event["data"])
    changed_vals=event["data"]
    
    
    
    
    #RUN AUTO MODE
    # traverse the path in the shortest way
    run=int(db.child('fk_values').child("run").get().val())
    mode=str(db.child('fk_values').child("mode").get().val())
    DEVICE=str(db.child('fk_values').child("device").get().val())
        
    print("RUN=",run,"MODE=", mode, "DEVICE=", DEVICE)
    
    
    
    if run!=0 and DEVICE=="APP":
        
        init_pos()
        
        if mode=="auto":
            print("Auto Mode")
            
            auto_m_thickness=int(db.child('fk_values').child("auto_value").child("thickness").get().val())
            auto_m_type=str(db.child('fk_values').child("auto_value").child("type").get().val())
    
    
            path = exec_get_traverse_path(auto_m_type.lower(), auto_m_thickness)
            traverse(path)
            
#             img = cv2.imread("output.jpg")
#             cv2.imshow("Image", img)
#             cv2.waitKey(300)
#             cv2.destroyAllWindows()
        elif mode=="manual":
            print("Manual Mode")
            #init_pos()
            m_theta1=int(db.child('fk_values').child("manual_value").child("theta1").get().val())
            m_theta2=int(db.child('fk_values').child("manual_value").child("theta2").get().val())
            m_Z=int(db.child('fk_values').child("manual_value").child("Z").get().val())
    
            rotate_to_angle1(motor1_direction_pin, motor1_pulse_pin, int(m_theta1))
            rotate_to_angle2(motor2_direction_pin, motor2_pulse_pin, int(m_theta2))
            rotate_to_angle3(motor3_direction_pin, motor3_pulse_pin, int(m_Z))
            
     
    
# Listen for database changes and call the update_gui function
db_stream_mode = db.child('fk_values').stream(mode_handler)



# def automode_handler(event):
#     
#     print("Auto Mode",event["data"])
#     changed_vals=event["data"]
#     auto_m_thickness=changed_vals["thickness"]
#     auto_m_type=changed_vals["type"]
#     
#     
#     #RUN AUTO MODE
#     # traverse the path in the shortest way
#     run=int(db.child('fk_values').child("run").get().val())
#     
#     init_pos()
#     
#     if run!=0:
#         print("RUN=",run)
#         path = exec_get_traverse_path(auto_m_type.lower(), auto_m_thickness)
#         traverse(path)
#         
#         img = cv2.imread("output.jpg")
#         cv2.imshow("Image", img)
#         cv2.waitKey(300)
#         cv2.destroyAllWindows()
        
    
# Listen for database changes and call the update_gui function
#db_stream_auto = db.child('fk_values').child("auto_value").stream(automode_handler)

# def manualmode_handler(event):
#     
#     print("Manual Mode",event["data"])
#     changed_vals=event["data"]
#     m_theta1=changed_vals["theta1"]
#     m_theta2=changed_vals["theta2"]
#     m_Z=changed_vals["Z"]
#     
#     #RUN MANUAL MODE
#     # traverse the path in the shortest way
#     run=db.child('fk_values').child("run").get()
#     
#     init_pos()
#     
#     if run!=0:
#         rotate_to_angle1(motor1_direction_pin, motor1_pulse_pin, int(m_theta1))
#         rotate_to_angle2(motor2_direction_pin, motor2_pulse_pin, int(m_theta2))
#         rotate_to_angle3(motor3_direction_pin, motor3_pulse_pin, int(m_Z))
    
# Listen for database changes and call the update_gui function
#db_stream_manual = db.child('fk_values').child("manual_value").stream(manualmode_handler)

# def update_toggle(event):
#     global DEVICE
#     print(event["data"])
#     device_data=event["data"]
#     if device_data=="APP":
#         DEVICE="APP"
# 
#     elif device_data=="PC":
#         DEVICE="PC"

    
    
    
# Listen for database changes and call the update_gui function
#db_stream2 = db.child('fk_values').child("device").stream(update_toggle)

while True:
    time.sleep(1)





