# Autonomous UWB Tracking Smart Luggage

**IEEE Fall 2024 Quarterly Project** | Oct 2024 - Dec 2024

## Overview

This repository contains the core ESP32/Arduino C++ implementation for an autonomous, follow-me smart luggage system. It utilizes Ultra Wide-Band (UWB) sensors for precise positioning and an Extended Kalman Filter to maintain accurate trajectory tracking.

*Note: This repository serves as a code portfolio showcasing the embedded systems logic, sensor fusion algorithms, and IoT communication protocols developed during the project.*

## Core Systems Architecture

1. **Sensor Fusion (`KalmanFilter_UWB.cpp`):** Integrated an Extended Kalman Filter on the ESP32 to process real-time UWB telemetry, continuously calculating 2D coordinates, heading, and velocity.
2. **Motor Actuation (`MotorControl_PID.cpp`):** Optimized a PID controller to translate navigation coordinates into smooth motor actuation and dynamic steering commands for autonomous maneuvering.
3. **IoT Communication (`BLE_Interface.cpp`):** Designed an IoT communication layer via BLE to interface the ESP32 with a smartphone, transmitting live telemetry data and receiving user navigation commands.

## Hardware Stack

* **Microcontroller:** ESP32
* **Actuators:** DC Motors with Encoders (9V supply)
* **Sensors:** UWB Transceivers
* **Environment:** Arduino IDE, C++
