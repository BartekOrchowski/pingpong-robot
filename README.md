# Ping-Pong Robot

An ESP32-based mobile robot controlled wirelessly using an Xbox controller. The robot combines differential drive locomotion with a dual-wheel ping-pong ball launcher and a servo-operated ball feeding mechanism.

The firmware is being developed in **C using ESP-IDF and FreeRTOS**, with an emphasis on modular software architecture, real-time control, and operational safety.

**Project status:** Early development — firmware architecture and hardware integration in progress.

## Project Overview

The robot is designed to perform three main functions:

- **Locomotion:** Two independently controlled DC motors provide forward, backward, and rotational movement.
- **Ball launching:** Two counter-rotating wheels, each driven by a DC motor, accelerate and launch ping-pong balls.
- **Ball feeding:** A gravity-fed magazine and servo-controlled feeder deliver individual balls to the launcher.

Horizontal aiming is achieved by rotating the entire robot chassis.

## Hardware

| Component | Description |
|---|---|
| Microcontroller | ESP32 DevKit V1 / ESP-WROOM-32  |
| Controller | Xbox wireless controller (compatibility to be verified) |
| Drive system | 2 DYNAMIXEL DC motors  |
| Launcher | 1 DC motors driving counter-rotating wheels |
| Feeder | Servo-operated mechanism |
| Communication | Bluetooth (protocol compatibility under investigation) |
| Power supply | Battery-powered system |

## Software Stack

- **Language:** C
- **Framework:** ESP-IDF
- **RTOS:** FreeRTOS
- **Version control:** Git and GitHub
- **Development environment:** Visual Studio Code

## Planned Software Architecture

The firmware will be divided into functional modules:

| Module | Responsibility |
|---|---|
| Bluetooth | Wireless controller communication and input processing |
| Drive Control | Differential drive motor control |
| Launcher Control | Launcher motor control |
| Feeder Control | Single-ball feeding mechanism |
| Safety Supervisor | Communication monitoring, command validation, and safe-state management |

FreeRTOS tasks, queues, and synchronization mechanisms will be introduced according to the timing and communication requirements of each subsystem.

A communication-loss failsafe is planned to stop the drive motors, disable the launcher, and inhibit ball feeding when controller communication is lost.


## Author

Developed by [BartekOrchowski](https://github.com/BartekOrchowski).

