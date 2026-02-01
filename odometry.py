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

def odometry(powers):
    count = 0
    values = {}
    for l, r in powers:
        left_motor.power_command = l
        right_motor.power_command = r

        values[count] = [(right_motor.position, right_motor.velocity), (left_motor.position, left_motor.velocity)]

        time.sleep(3)

    linear_velocity = 0
    x, y = 0
    theta = 0
    omega = 0
    r = 1.125
    wheel_base = 5.5
    position = [x, y, theta]

    for i in range(3):
        right_velocity = values[count][0][1]
        left_velocity = values[count][1][1]
        velocity = (right_velocity+left_velocity)/2

        omega = (r*right_velocity-r*left_velocity)/wheel_base

        k_00 = velocity*math.cos(theta)
        k_01 = velocity*math.sin(theta)
        k_02 = omega

        k_10 = velocity*math.cos(theta+3/2*k_02)
        k_11 = velocity*math.sin(theta+3/2*k_02)
        k_12 = omega

        k_20 = velocity*math.cos(theta+3/2*k_12)
        k_21 = velocity*math.sin(theta+3/2*k_12)
        k_22 = omega

        k_30 = velocity*math.cos(theta+3*k_22)
        k_31 = velocity*math.sin(theta+3*k_22)
        k_32 = omega

        x = x+3/6(k_00+2(k_10+k_20)+k_30)
        y = y+3/6(k_01+2(k_11+k_21)+k_31)
        theta = theta+3/6(k_02+2(k_12+k_22)+k_32)
    
    return [x, y, theta]

print(odometry([[0.3, 0.75], [-0.5, 0.5], [-0.3, 0]]))