# Welcome to the Design 2 Drone Project by Team 02! 😊

This repository contains the source code for implementing key drone functionalities, including:

- **Line Detection**
- **Autonomous Takeoff**
- **Hovering**
- **Landing**


## Features

- **Line Detection**: Detects and follows lines using the ESP32 CAM with MAVLink integration.
- **Autonomous Takeoff**: Enables the drone to take off automatically without manual intervention.
- **Hovering**: Utilizes an optical flow sensor for stable and controlled hovering.
- **Landing**: Automated landing sequence initiated after hovering.

## Technology Stack

- **DroneKit** for flight control
- **MAVLink** for communication with the flight controller
- **ESP32 CAM** for vision-based line detection
- **Optical Flow Sensor** for hover stability
- **PI Control** for motor efficiency
- **HC-SR04 Ultrasonic sensor for obstacle avoidance

# Design Output folder
1. Datasheets
2. Documents
3. Test Results
4. Firmware
5. 3D Modelling

## Getting Started

To clone the repository:

```bash
git clone https://github.com/Pinaki24/Design2.git
```
Installing dependencies
```bash
pip install dronekit
pip install dronekit-sitl
```
## Installing Python 3.6.8

For **Windows**, download and install Python 3.6.8 from the [official Python website](https://www.python.org/ftp/python/3.6.8/python-3.6.8-amd64.exe).


# Additional libraries:

```bash
pip install opencv-python
pip install numpy
```

