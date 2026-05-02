# Gesture Controlled Robotic Arm

## Overview

This project implements a gesture-controlled robotic arm using ESP32 microcontrollers and MPU6050 sensors. Hand movements are captured using a glove and transmitted wirelessly via ESP-NOW to control the arm in real time.

---

## Components

* ESP32 ×2
* MPU6050 sensors ×2
* TCA9548A I2C multiplexer
* PCA9685 servo driver
* Servo motors (MG996R, MG90S)
* Stepper motor (base)
* Buck converter
* Push button (gripper control)

---

## Working

The system consists of a transmitter and a receiver.

The transmitter reads motion data from the MPU6050 sensors and sends it wirelessly using ESP-NOW. The receiver processes this data and converts it into PWM signals to control the motors of the robotic arm.

Flow:
MPU6050 → ESP32 (Transmitter) → ESP-NOW → ESP32 (Receiver) → PCA9685 → Motors

---

## Code Structure

* `firmware/transmitter_glove/` → sensor reading and data transmission
* `firmware/receiver_arm/` → motor control and data processing

---

## Documentation

Detailed explanations, circuit diagrams, and design decisions are available in:

docs/project-report.pdf

---

## Notes

* A push button was used instead of a flex sensor for more reliable gripper control
* One joint was removed to improve stability
* Servo motors do not provide feedback, so the system assumes an initial reference position
