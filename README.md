# Design and Development of a Vision based 4 DOF SCARA Robot for Screw Operations

A 4 DOF SCARA robot was designed and developed for screw operations. 

YOLO v4 was used to train the screw heads and holes detection model. After filtering out the center coordinates of the screw heads, it was transformed from camera frame to robot base frame and then the required joint angles to rotate will be identified by solving inverse kinematics equations. Hence, the manipulator could autonomously move onto the screw heads after detecting the positions. Raspberry Pi 3B+ was the main controller, and a mobile application was developed to switch the modes from manual to auto and vice versa.

## Features
• SCARA with vision-based screw detection to locate screw heads & perform screw loosing operations<br>
• Forward and inverse kinematics, robot dynamics, control system design (Disturbance Observer), machine learning.<br>
• Technologies: SOLIDWORKS, ADAMS, MATLAB, Yolo V4, Python, Ansys Mechanical, Raspberry pi.

[![Video Demo](https://img.youtube.com/vi/ghEVJmXheUc/0.jpg)](https://www.youtube.com/watch?v=ghEVJmXheUc)
