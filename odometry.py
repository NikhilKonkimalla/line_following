import numpy as np
import time
import board
import math
import adafruit_bh1750
from motorgo import Plink, ControlMode

i2c = board.I2C()

sensor = adafruit_bh1750.BH1750(i2c)

# Initialize Plink board
plink = Plink()
plink.power_supply_voltage = 9.6  # Set to our battery voltage

# Set up motor channels - 1 motor per side
left_motor = plink.channel1
right_motor = plink.channel2

# Set motor voltage limits (default is 0)
left_motor.motor_voltage_limit = 6.0
right_motor.motor_voltage_limit = 6.0

# Connect to the board
plink.connect()

# Set control mode to POWER (i.e. simple)
left_motor.control_mode = ControlMode.POWER
right_motor.control_mode = ControlMode.POWER

def odometry(powers, sample_time=3.0, dt=0.1):
    # Wheel/robot geometry (inches)
    wheel_radius = 1.125
    wheel_base = 5.5

    x, y, theta = 0.0, 0.0, 0.0

    for l, r_cmd in powers:
        left_motor.power_command = l
        right_motor.power_command = -r_cmd

        start_time = time.monotonic()
        while time.monotonic() - start_time < sample_time:
            left_w = left_motor.velocity   # rad/s
            right_w = -right_motor.velocity # rad/s

            # Print live motor velocities
            print(
                f"Motor velocities | Left: {left_w:.2f} rad/s | Right: {right_w:.2f} rad/s"
            )

            # Convert wheel angular velocity to linear velocity at the ground
            v = wheel_radius * (right_w + left_w) / 2.0
            omega = (wheel_radius * (right_w - left_w)) / wheel_base

            # Simple Euler integration for odometry
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += omega * dt

            time.sleep(dt)

    return [x, y, theta]

print(odometry([[1, 1], [1, 1], [1, 1]]))
