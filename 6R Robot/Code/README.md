# Robotastic FOC Stepper Controller

This project provides a Field-Oriented Control (FOC) implementation for stepper motors, specifically designed for STM32G4 microcontrollers. It leverages the SimpleFOC library to achieve smooth, high-torque, and high-efficiency motor operation.

## Features
- **Field-Oriented Control:** Advanced motor control for quiet and precise movement.
- **CAN-Bus Communication:** Fully integrated FDCAN interface for remote control and telemetry.
- **Motion Profiles:** Built-in trapezoidal and triangular ramp generators for smooth acceleration and deceleration.
- **Advanced State Machine:** Supports multiple modes including Position, Velocity, Torque, and Homing sequences.
- **Open-Loop Mode:** Useful for testing or applications where sensor feedback is not required for specific tasks.

## Hardware Requirements
- **Microcontroller:** STM32G4 series or compatible.
- **Driver:** 4-PWM Stepper Driver.
- **Sensor:** Magnetic Sensor (AS5047 via SPI).
- **Feedback:** Support for endstop pins for homing.

## Getting Started (PlatformIO)
This project is built using **PlatformIO**. It is the recommended environment for development.

### How to load the project:
1. **Install VS Code:** Download and install Visual Studio Code.
2. **Install PlatformIO:** Search for the "PlatformIO IDE" extension in the VS Code marketplace and install it.
3. **Open Project:**
   - Clone this repository or download the source code.
   - In PlatformIO Home, click on "Open Project" and select the folder containing `platformio.ini`.

## Configuration
Main configurations like CAN IDs and hardware pins are centralized in `main.cpp` and `Board.h`.

## License
This software is provided for **educational and personal use only**. 
**Commercial use is strictly prohibited** without prior written permission from the author (Robotastic).