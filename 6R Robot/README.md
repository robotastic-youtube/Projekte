# Robotastic 6R-Robot & FOC Controller

This repository contains all the necessary resources for building and controlling a 6-axis robotic arm (6R-Robot), along with the custom electronics and firmware. The project relies on a hardware abstraction layer and Field-Oriented Control (FOC) for precise stepper motor operation.

## ⚠️ IMPORTANT SAFETY WARNING & DISCLAIMER ⚠️

This project involves building and operating a **6-axis robot** driven by stepper motors and custom electronics. Please strictly observe the following points before attempting to build or operate it:

* **Mechanical Hazard:** A robotic arm can move unpredictably, at high speeds, and with significant force—especially during programming, FOC parameter tuning, or due to unforeseen software bugs. Always maintain a safe distance during operation and **never** reach into the robot's workspace while it is powered on!
* **Electrical Hazard:** This system operates with voltages and high-current power supplies (e.g., 24V) that can cause fires, severe injuries, or the destruction of hardware if wired incorrectly, short-circuited, or overloaded. Adequate foundational knowledge of electronics is strictly required.
* **Not an Industrial Product:** This is an experimental DIY and educational project. It does not carry a CE mark and lacks the certified safety mechanisms (such as verified emergency stop chains or light curtains) found in commercial industrial robots.

**Use at Your Own Risk:**
Building, programming, and operating this robot is done **entirely at your own risk**. The author (Robotastic) assumes absolutely no liability for any personal injury, property damage, or financial loss resulting from the construction, use, or potential malfunction of this project.

## 1. Folder Structure

The project is organized into three main directories:

* **`Electronics/`**: Contains the **KiCad files** for the custom FOC PCB designed to drive the stepper motors.
* **`Code/`**: Contains the **Firmware** for the PCBs. This code is built using PlatformIO and implements hardware initialization, CAN-bus communication, and the FOC algorithm.
* **`3D-Files/`**: Contains the mechanical designs in **STEP format**.
    * Individual joints are provided as assemblies named `Joint1` through `Joint6`.
    * The file `6R-Robot Assembly` contains the complete, assembled robot model.

## 2. Documentation & Build Guide

For detailed videos on how to assemble the robot, explanations of the electronics, and the software, please visit the **Robotastic YouTube Channel**:

👉 [Robotastic YouTube Channel](https://www.youtube.com/channel/UCKuV_Eh0zMl3OSophDiDIqA)

## 3. Current Project Status (Hardware)

Please note that this project is currently a **Work in Progress (WIP)**.
* There is currently **no official Bill of Materials (BOM)** or detailed guide available.
* However, the robot can still be built, as the necessary components can be inferred from the provided documentation, PCB designs, and 3D models.

## 4. Software Status

The firmware provided in the `Code/` folder is designed for the direct low-level control of the motor drivers:
* **Currently Implemented**: Low-level drivers, FOC logic, CAN communication, and simple motion commands (Position, Velocity, Torque, Trajectory points).
* **Not Yet Implemented**: There is currently **no high-level software** for path planning (such as Inverse Kinematics or full-arm trajectory generation) or a graphical user interface (GUI) to control the robot as a complete system.

## 5. Getting Started (PlatformIO)

This project is built using **PlatformIO**, which is the recommended development environment.

### How to load the project:
1. **Install VS Code:** Download and install Visual Studio Code.
2. **Install PlatformIO:** Search for the "PlatformIO IDE" extension in the VS Code marketplace and install it.
3. **Open Project:**
   - Clone this repository or download the source code.
   - In PlatformIO Home, click on "Open Project" and select the folder containing `platformio.ini`.
4. **Build & Upload:**
   - Connect your STM32 board via USB (ensure you have the correct programmer, e.g., ST-Link).
   - Use the PlatformIO toolbar (bottom bar) to click the **Build** (Checkmark) and then **Upload** (Arrow) icons.

## License

This software, hardware design, and mechanical design are provided for **educational and personal use only**. 
**Commercial use is strictly prohibited** without prior written permission from the author. 

Please see the `LICENSE` file for the complete Robotastic Non-Commercial License terms.

*This project relies on the excellent SimpleFOC library, which is licensed under the MIT License.*