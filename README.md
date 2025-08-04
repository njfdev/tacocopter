<div align="center">
  <img style="width: 36rem;" alt="Tacocopter Mid-Flight Image" src="https://github.com/user-attachments/assets/a1dcea42-80fc-410e-b058-85001ea650b9" />
</div>

# Tacocopter 🌮

Starting from a personal goal set at the beginning of 2025 to build and program a drone in Rust from scratch, Tacocopter is the loving name for the project as a whole.

- Programmed *from scratch* in Rust for a Raspberry Pi Pico 2
- Custom flight controller hardware design assembled onto a breadboard
- A desktop app in Rust to easily interact with the flight controller

## Overview
There are 2 main applications in this project. The `firmware` directory contains the code flashed to the drone's microcontroller and `configurator` is a desktop application to configure and view info about the drone. This is a complex project, so there are quite a few parts.

> P.S. There are **so many** awesome implementation details I don't have room to share about, but might discuss in a future video. For example, writing custom PIO assembly programs (special to the Raspberry Pi Picos) for efficient I/O access and all the caveats I worked around to get the flash working! :)

`firmware`:
- Designed for a Raspberry Pi Pico 2 (`rp2350`) microcontroller overclocked to 300MHz
- Programmed with the [Embassy](https://github.com/embassy-rs/embassy) framework
- Control loop running at 500Hz
- Supports rate/acro control (think manual FPV-style flying)
- Supports ELRS controller with drone telemetry
  - Battery percentage/voltage reporting
  - Altitude reporting
  - GPS positioning
- Various Sensors
  - MPU6050 IMU for determining drone orientation
  - BMP390 barometer for determining altitude
  - HC-SR04 ultrasonic sensor for height above ground
  - HGLRC M100-5883 GPS for global positioning
  - PM02D power module for battery monitoring
  - RadioMaster RP3 ELRS module to receive controller signals
- DShot support for sending commands to motors/ESCs
- Onboard persistent logging of flight data (Blackbox)
- Realtime console-like logging
- Persistent storage of settings in flash


`configurator`:
- Desktop app programmed in Rust to configure the drone flight controller
- Programmed with the [Tauri V2](https://v2.tauri.app/) framework
- Supports viewing important sensor/statistic info from the drone:
  - Uptime
  - IMU data
  - Ultrasonic sensor data
  - ELRS controller channel outputs
  - Realtime logs from the flight controller
- Easy gyroscope calibration interface
- Interface to view and set the PID values
- Interface to enable and download Blackbox logs


`tc_interface`:
- A small library containing data structures shared between the `firmware` and `configurator`
- Mainly for having a common interface for serializing and deserializing data sent over the USB connection

## Usage

⚠️ Please note: This project has not yet been designed for use by others, so don't expect precompiled applications or fully functional features. Descriptions to set up the hardware are not provided.

First, clone the repository in a convienent folder:
```bash
git clone https://github.com/njfdev/tacocopter.git
cd tacocopter
```

In order to compile the drone and flash the firmware, first plug in the Raspberry Pi Pico 2 while holding the `BOOTSEL` button to enter the flashing mode. You can then run these commands.
```bash
cd firmware
cargo run --release
cd ../
```

To run the configurator application, you can either run it in development mode (which I usually do) or compile and build it.
```bash
cd configurator

# for development mode
cargo tauri dev

# for compiling
cargo tauri build
```
To use the configurator, make sure the app is open and plug in a USB cable connected to the drone (WITHOUT holding BOOTSEL).
