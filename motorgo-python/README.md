# MotorGo Python

The official Python API for controlling PiHat-compatible MotorGo boards from the Raspberry Pi.

## Installation

To install the `pyplink` package, you can use `pip` once you have cloned this repository

```sh
git clone https://github.com/Every-Flavor-Robotics/pyplink.git
cd pyplink
pip install pyplink
```

### Setting up the Plink
First, you need to flash the Plink with the PyPlink firmware. This firmware is located in the `pyplink_firmware` directory of this respository. You can flash the firmware using either platformio or the Arduino IDE.

Once the firmware is flashed, you can connect the Plink to the Raspberry Pi using the GPIO headers. At this point, the Plink is ready to be used.

## Usage
Below is an example program that demonstrates how to use the pyplink package to control motor channels on the MotorGo Plink board.

This example can be found in the `examples/` directory of this repository.
```python
# spin_motors.py
# Before running this script, ensure that the MotorGo Plink is
# connected to the Raspberry Pi and that it has been flashed with the
# MotorGo firmware.

from pyplink import Plink, BrakeMode, ControlMode
import time


def main():
    # Create a Plink object
    plink = Plink()

    # This command will initiate communications and confirm
    # that the Plink is connected/available
    plink.connect()

    # The Plink object has 4 MotorChannel objects, corresponding to the 4 motor channels
    # on the board
    # You can save references as local variables for convenience (as below) or
    # access them directly from the Plink object
    left_motor = plink.channel1
    right_motor = plink.channel2

    # You can configure the control mode of the motor channels
    # The two options are ControlMode.VELOCITY and ControlMode.POWER
    left_motor.control_mode = ControlMode.VELOCITY
    right_motor.control_mode = ControlMode.VELOCITY
    plink.channel3.control_mode = ControlMode.POWER
    plink.channel4.control_mode = ControlMode.POWER

    # You can configure the brake mode of the motor channels
    # The two options are BrakeMode.BRAKE and BrakeMode.COAST
    left_motor.brake_mode = BrakeMode.BRAKE
    right_motor.brake_mode = BrakeMode.BRAKE
    plink.channel3.brake_mode = BrakeMode.COAST
    plink.channel4.brake_mode = BrakeMode.COAST

    # Main program loop
    velocity_command = 0.0
    while True:

        # Set the velocity command in rad/s
        left_motor.velocity_command = velocity_command
        right_motor.velocity_command = -velocity_command

        # Ramp up the velocity command
        velocity_command += 0.1
        if velocity_command > 5.0:
            velocity_command = -5.0

        # Set the power command in the range [-1, 1]
        plink.channel3.power_command = 0.5
        plink.channel4.power_command = -0.5

        # You can read the position and velocity of the motor channels
        print("----")
        print(
            f"Channel 1 position: {plink.channel1.position}, velocity: {plink.channel1.velocity}"
        )
        print(
            f"Channel 2 position: {plink.channel2.position}, velocity: {plink.channel2.velocity}"
        )
        print(
            f"Channel 3 position: {plink.channel3.position}, velocity: {plink.channel3.velocity}"
        )
        print(
            f"Channel 4 position: {plink.channel4.position}, velocity: {plink.channel4.velocity}"
        )
        print("-----")

        # Delay as long as you need, communications continue in the background
        time.sleep(0.1)


if __name__ == "__main__":
    main()
```



