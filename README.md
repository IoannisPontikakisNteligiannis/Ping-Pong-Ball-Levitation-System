# Ping-Pong-Ball-Levitation-System Using PID Control
A PID-controlled system that uses air flow to levitate and maintain a ball at a specified height. Built with Arduino and real-time position control using ultrasonic sensing.

## Project Overview
This project implements a closed-loop control system for levitating a ball using air flow. The system uses PID (Proportional-Integral-Derivative) control to maintain a ball at a desired height by adjusting the air flow from a motor. Real-time position feedback is provided by an ultrasonic distance sensor, with user interaction through an LCD display and potentiometer for setpoint adjustment.
# System Components

## Hardware: <br>
Arduino Uno/Nano microcontroller  <br>
HC-SR04 ultrasonic distance sensor <br>
16x2 LCD display <br>
DC motor with PWM control <br>
Potentiometer for setpoint control <br>
Push button for mode selection <br>
Buzzer for audio feedback <br>
Custom acrylic tube housing <br>
Power supply unit <br>

## Software Dependencies: <br>
Arduino IDE <br>
LiquidCrystal library <br> 
Python with libraries:
pyserial
matplotlib
numpy

# Technical Specifications

## PID Controller Parameters:
Proportional Constant (KP): 4.0  <br>
Integral Constant (KI): 1.5  <br>
Derivative Constant (KD): 1.0  <br>
Sample Time: 0.05 seconds  <br>
Output Range: 50-255 PWM  <br>

## Operating Ranges:
Distance Measurement: 2-400 cm <br>
Target Position Range: 0-55 cm <br>
Control Update Rate: 1 ms <br>
Display Update Rate: 320 ms <br>

# Setup Instructions

## Hardware Assembly:
Mount the ultrasonic sensor at the top of the acrylic tube <br>
Attach the DC motor to provide upward airflow <br>
Connect the LCD display and control components <br>
Ensure stable mounting of all components <br>

## Wiring Configuration

![image](https://github.com/user-attachments/assets/2b57421c-da57-491c-a4bc-7cc2c1464240)

## Software Installation

Upload the Arduino code to your microcontroller <br>
Install required Python libraries: pip install pyserial matplotlib numpy <br>
Adjust the COM port in the Python script to match your system <br>

## Operation Modes

INIT Mode: 
System startup state <br>
Displays welcome message <br>
Press button to proceed <br>


SET_POSITION Mode:
Use potentiometer to set desired ball height <br>
Current target height shown on LCD <br>
Press button to start control   <br>


CONTROL Mode: 
Active PID control maintains ball position  
Real-time position and target displayed  <br>
Press button to return to SET_POSITION  <br>

## Challenges and Solutions

Air Flow Control:
Challenge: Non-linear relationship between motor PWM and air flow  <br>
Solution: Implemented minimum PWM threshold and tuned PID parameters  <br>


Position Sensing:
Challenge: Ultrasonic sensor noise and false readings  <br>
Solution: Added error checking and signal filtering  <br>


System Stability:
Challenge: Ball oscillations at certain heights  <br> 
Solution: Fine-tuned PID parameters and implemented derivative control  <br>


## System Behaviour: 
<img width="1920" height="975" alt="image" src="https://github.com/user-attachments/assets/13ea7adf-73c5-4f41-ba82-ed0b69e04f4b" />



## Future Improvements: 
Auto-tuning PID parameters  <br>
Wireless control interface  <br>
Additional control algorithms (e.g., fuzzy logic)  <br>
Enhanced data logging and analysis  <br>
3D-printed custom housing  <br>

## TINKERCAD SIMULATION
https://www.tinkercad.com/things/ij7YmJMZmQy-air-ball-levitation-system?sharecode=pIKio825akA3C3FJb9IXXrqDX-6_rH5UZpds-5RQVkQ

