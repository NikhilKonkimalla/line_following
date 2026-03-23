<h1 align="center">📘 | MotorGo Plink Tools for CMU 16-311 | 📘 </h1>

<p align="center">
  Tools for the MotorGo Plink Education Kit, customized for 16-311
</p>

---

## Table of Contents
- [Table of Contents](#table-of-contents)
- [Introduction](#introduction)
- [Installation](#installation)
- [Example Code:](#example-code)
- [Setup](#setup)
- [VNC](#vnc)

---


## Introduction

This repository compiles all of the software for setting up the MotorGo Plink Education Kit for the 16-311 projects:

* Custom MotorGo Python driver
  * Locked voltage limits MotorGo Red motors
  * Locked power supply voltage
* VL53L4CX range sensor driver
* BH1750 light sensor driver



## Installation
We recommend using the latest Rasbperry Pi OS on your Raspberry Pi. You can setup Raspberry Pi OS using the RPi imager, available [here](https://www.raspberrypi.com/software/). Once the operating system is setup, you can install all of the tools by running this command

``` bash
curl -s https://raw.githubusercontent.com/Every-Flavor-Robotics/cmu-16311-tools/refs/heads/main/install.sh | bash
```

## Example Code:
The examples directory contains examples for using the various features of the MotorGo Plink Education Kit:

* Controlling motors
* Reading data from the onboard IMU
* Reading data from the light and range sensors

## Setup

After completing installation, you need to enable SPI, I2C, and VNC using raspi-config. Run:

```bash
sudo raspi-config
```

Navigate to `3 Interface Options`, enable VNC, SPI, and I2C.



## VNC

Follow the setup steps [here](https://www.raspberrypi.com/documentation/computers/remote-access.html#connect-to-a-vnc-server) to setup the VNC client.