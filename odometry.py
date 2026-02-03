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

def odometry(powers, sample_time=3.0, dt=0.07, start_threshold=0.0001, ma_window=10):
    # Wheel/robot geometry (inches)
    wheel_radius = 1.1
    wheel_base = 5.25
    right_sign = -1.0  # Set to -1.0 if right encoder direction is reversed

    x, y, theta = 0.0, 0.0, 0.0
    left_hist = []
    right_hist = []

    for l, r_cmd in powers:
        left_motor.power_command = l
        right_motor.power_command = -r_cmd

        # Wait for motors to start moving (avoid integrating zeros)
        while True:
            left_w = left_motor.velocity
            right_w = right_sign * right_motor.velocity
            if abs(left_w) >= start_threshold or abs(right_w) >= start_threshold:
                break
            time.sleep(0.01)

        start_time = time.monotonic()
        while time.monotonic() - start_time < sample_time:
            left_w = left_motor.velocity   # rad/s
            right_w = right_sign * right_motor.velocity # rad/s (sign-corrected)

            # Moving average filter for encoder velocities
            left_hist.append(left_w)
            right_hist.append(right_w)
            if len(left_hist) > ma_window:
                left_hist.pop(0)
                right_hist.pop(0)
            left_w = sum(left_hist) / len(left_hist)
            right_w = sum(right_hist) / len(right_hist)

            # Print live motor velocities
            print(
                f"Motor velocities | Left: {left_w:.2f} rad/s | Right: {right_w:.2f} rad/s"
            )

            # Convert wheel angular velocity to linear velocity at the ground
            v = wheel_radius * (right_w + left_w) / 2.0
            omega = (wheel_radius * (right_w - left_w)) / wheel_base

            # Runge-Kutta 4th order integration (RK4)
            k1_x = v * math.cos(theta)
            k1_y = v * math.sin(theta)
            k1_t = omega

            k2_x = v * math.cos(theta + 0.5 * dt * k1_t)
            k2_y = v * math.sin(theta + 0.5 * dt * k1_t)
            k2_t = omega

            k3_x = v * math.cos(theta + 0.5 * dt * k2_t)
            k3_y = v * math.sin(theta + 0.5 * dt * k2_t)
            k3_t = omega

            k4_x = v * math.cos(theta + dt * k3_t)
            k4_y = v * math.sin(theta + dt * k3_t)
            k4_t = omega

            x += (dt / 6.0) * (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x)
            y += (dt / 6.0) * (k1_y + 2.0 * k2_y + 2.0 * k3_y + k4_y)
            theta += (dt / 6.0) * (k1_t + 2.0 * k2_t + 2.0 * k3_t + k4_t)

            time.sleep(dt)
        left_motor.power_command = 0
        right_motor.power_command = 0
        time.sleep(0.5)

    return [x, y, theta]

print(odometry([[1, 1]]))
