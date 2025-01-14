# Carver_Interface

## Introduction

The **Carver Interface** is a versatile and user-friendly Human-Machine Interface (HMI) designed for seamless interaction with the **Carver** autonomous and teleoperated vehicle. It provides a range of control options and operational flexibility, enabling users to interact with the system effectively. Whether you want to manually control the vehicle, operate it remotely, or let it navigate autonomously, the Carver Interface offers the tools to do so efficiently. This document outlines the interface’s features, modes, and how it works.

---

## Table of Contents

1. [Overview of the Carver Interface](#overview-of-the-carver-interface)
2. [Modes of Operation](#modes-of-operation)
   - [Steering Mode](#steering-mode)
   - [Teleoperation (Teleop) Mode](#teleoperation-teleop-mode)
   - [Joystick Mode](#joystick-mode)
   - [Auto Mode](#auto-mode)
3. [Interaction and Control Features](#interaction-and-control-features)
   - [Accelerator and Brake](#accelerator-and-brake)
   - [Lighting Control](#lighting-control)
4. [State Monitoring and Diagnostics](#state-monitoring-and-diagnostics)
5. [How the Interface Works](#how-the-interface-works)
6. [Future Enhancements](#future-enhancements)
7. [Getting Started](#getting-started)

---

## Overview of the Carver Interface

The Carver Interface serves as the central hub for interacting with the Carver vehicle. Designed with user experience in mind, it provides multiple operation modes tailored to various needs and scenarios. This makes the Carver system adaptable, whether you are controlling it manually, remotely, or allowing it to drive autonomously.

---

## Modes of Operation

### Steering Mode
- **Purpose**: Designed for manual control, this mode is ideal for situations where the user wants full hands-on operation.
- **Features**:
  - Users can twist the **hand accelerator** to increase speed and use the brake to slow down.
  - Forward and Backward buttons allow easy switching between driving directions.
  - Provides precise and responsive control for navigating tight spaces or complex environments.

---

### Teleoperation (Teleop) Mode
- **Purpose**: Created for remote control via a web interface, this mode is best for monitoring and controlling Carver from a distance.
- **Features**:
  - Users can access Carver's interface through a browser and operate the vehicle remotely.
  - Control the steering motor to turn left or right.
  - Adjust the speed using the **Brushless motor**.
  - Perfect for situations where direct manual control is not possible, such as hazardous or hard-to-reach locations.

---

### Joystick Mode
- **Purpose**: This mode enables remote operation using a joystick, ideal for scenarios requiring dynamic and precise control.
- **Features**:
  - Users can operate the steering motor and adjust speed via the **Brushless motor** using the joystick.
  - Provides a tactile and intuitive way to interact with Carver.
  - Suitable for applications like field navigation or interactive demonstrations.

---

### Auto Mode
- **Purpose**: Designed for fully autonomous operation, this mode is perfect for long-distance navigation or routine tasks.
- **Features**:
  - Users can set a destination on the map, and Carver will automatically navigate to it.
  - Employs advanced obstacle avoidance and route optimization.
  - Ideal for tasks requiring repetitive or predictable movement paths, such as delivery or patrol routes.

---

## Interaction and Control Features

### Accelerator and Brake
- **Hand Accelerator**:
  - Allows users to increase Carver’s speed by twisting the accelerator handle.
  - Designed for smooth and precise speed control.
- **Steering Brake**:
  - Enables users to slow Carver down as required.
  - Offers safe and responsive braking for better maneuverability.

---

### Lighting Control

The Carver Interface incorporates an intelligent and dynamic lighting control system for enhanced visibility and safety. Below are the detailed functionalities:

1. **Turn Signal Lights**:
   - The turn signal lights automatically activate in the direction of the turn or the direction the vehicle is about to turn.
   - This feature operates seamlessly across all operation modes (Steering Mode, Teleop Mode, Joystick Mode, and Auto Mode).

2. **Front Lights**:
   - The front lights have two levels of brightness for different scenarios:
     - **Dim Light**: For normal visibility during low-light conditions.
     - **High Beam**: For enhanced visibility in darker environments or when additional lighting is needed.
   - Users can manually control the front lights (toggle between on/off and dim/high beam) via the interface.

3. **Rear Lights**:
   - The rear lights also have two levels:
     - **Dim Light**: Always on by default to indicate the presence of the vehicle.
     - **Brake Light**: Activates automatically under the following conditions:
       - When the hand brake is engaged.
       - When braking is performed via the Brushless motor.
   - This dynamic system ensures other vehicles or users are alerted to braking actions for safety.

---

## How the Interface Works

The Carver Interface operates through a combination of hardware and software systems:
- **Hardware Integration**: Sensors and actuators are connected to the central control unit for seamless data flow.
- **Web Interface**: Allows remote control and monitoring via a browser.
- **Joystick Connectivity**: Provides a wireless connection for dynamic control.
- **Autonomous Navigation**: Combines mapping, LiDar, and obstacle avoidance for efficient auto-navigation.

---

## Getting Started



---
