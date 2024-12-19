# Welcome to the UAV Project Repository!

This project focuses on the development and testing of a fixed-wing, single-propeller Unmanned Aerial Vehicle (UAV). The repository contains firmware for the flight controller, test files for telemetry, a prototype ground station app, and MATLAB scripts for UAV modeling and joystick testing. Below is an overview of the repository contents and instructions for getting started.

## Repository Contents

### Firmware for Flight Controller
This directory contains the firmware files necessary to run the flight controller on the UAV. The firmware is responsible for controlling the UAV's flight dynamics and ensuring stable flight.

### Telemetry Test Files
These files are used to test the telemetry system of the UAV. The telemetry system is crucial for transmitting flight data from the UAV to the ground station.

### Prototype Ground Station App
Developed using MATLAB App Designer, this app serves as the interface for monitoring and controlling the UAV during flight. This directory includes the app files and the current development progress. The app is still under construction.

### MATLAB Scripts for UAV Model
This set of scripts is used for modeling the UAV. It includes simulations of the UAV's flight dynamics and performance analysis, enabling better understanding and optimization of the UAV's behavior.

### Joystick Testing Scripts
These MATLAB scripts are used to test joystick inputs for controlling the UAV. The scripts assist in configuring and calibrating the joystick for precise control during operation.

## Getting Started

### Prerequisites

- **MATLAB** with App Designer
- A compatible flight controller hardware
- A joystick for testing
- Basic knowledge of UAV systems and MATLAB

### Installation

#### 1. Set Up the Flight Controller
- Follow the instructions in the `firmware` directory to upload the firmware to your flight controller hardware.

#### 2. Telemetry Test Setup
- Navigate to the `telemetry` directory and follow the instructions to run the telemetry tests.
- Ensure that your telemetry hardware is properly connected and configured.

#### 3. Run the Ground Station App
- Open MATLAB and navigate to the `ground_station_app` directory.
- Open the app file (`GroundStationApp.mlapp`) in MATLAB App Designer.
- Run the app to start the ground station interface.

#### 4. UAV Model Simulation
- Navigate to the `uav_model` directory.
- Open and run the MATLAB scripts to simulate the UAV's flight dynamics.
- Adjust parameters as needed for your specific UAV model.

#### 5. Joystick Testing
- Connect your joystick to your computer.
- Navigate to the `joystick_testing` directory.
- Run the MATLAB scripts to test and calibrate the joystick inputs.

## Usage

- **Ground Station App**: Use the app to monitor and control the UAV during flight. It provides a user-friendly interface for real-time telemetry data and control inputs.
- **UAV Model Scripts**: Simulate and analyze flight performance before conducting actual flights.
- **Joystick Testing Scripts**: Ensure your joystick is accurately calibrated for controlling the UAV.

## License

This project is licensed under the MIT License - see the `LICENSE` file for details.

---
This repository provides a comprehensive toolkit for UAV development and testing. Contributions and feedback are welcome to improve and expand the project's capabilities.
