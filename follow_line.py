import numpy as np
import time
import board
import adafruit_bh1750
from motorgo import Plink, ControlMode

i2c = board.I2C()

sensor = adafruit_bh1750.BH1750(i2c)

# Initialize Plink board
plink = Plink()
plink.power_supply_voltage = 9.0  # Set to our battery voltage

# motor channels
left_motor_front = plink.channel1
left_motor_back = plink.channel2
right_motor_front = plink.channel3
right_motor_back = plink.channel4

# Set motor voltage limits (default is 0)
left_motor_front.motor_voltage_limit = 6.0
left_motor_back.motor_voltage_limit = 6.0
right_motor_front.motor_voltage_limit = 6.0
right_motor_back.motor_voltage_limit = 6.0

# Connect to the board
plink.connect()

# Set control mode to POWER (i.e. simple)
left_motor_front.control_mode = ControlMode.POWER
left_motor_back.control_mode = ControlMode.POWER
right_motor_front.control_mode = ControlMode.POWER
right_motor_back.control_mode = ControlMode.POWER

#Calibration on white
print("BEGIN CALIBRATION WHITE")
time.sleep(3)
#This might need to be a for loop if sensor 
# readings don't change between iterations
whiteCal = [sensor.lux for _ in range (20)]
meanWhite = sum(whiteCal) / len(whiteCal)

#Calibration on black
print("BEGIN CALIBRATION BLACK")
time.sleep(3)
#This might need to be a for loop if sensor 
# readings don't change between iterations
blackCal = [sensor.lux for _ in range(20)] 
meanBlack = sum(blackCal) / len(blackCal)

#Caluculate Threshold
edgeThreshold = (meanBlack + meanWhite) / 2
print(f"Average White reading: {meanWhite}")
print(f"Average Black reading: {meanBlack}")
print(f"Resulting edge threshold: {edgeThreshold}")
print("Beginning course traversal in 5 seconds:")
time.sleep(5)

#TUNING INSTRUCTIONS
#--------------------------------------------------------------#
# gain: Controls how aggressively robot corrects (0.1 - 0.5 recommended)
# base_speed: Base forward speed (0.0 - 1.0)
# correction_limit: Maximum correction to prevent over-steering
# Sample size: Increase if calibration is inconsistent
#--------------------------------------------------------------#

# PID Control parameters
gain = 0.1              # Proportional gain (start low, increase if sluggish)
base_speed = 0.3        # Base forward speed (0.0 to 1.0)
correction_limit = 0.2  # Max correction to prevent wild turns

print("Starting line following...")

try:
    while True:
        luxSample = sensor.lux
        error = luxSample - edgeThreshold
        correction = gain * error
        
        #over-steering prevention
        correction = max(-correction_limit, min(correction_limit, correction))
        
        # Positive correction = too bright = turn left (slow down left motor)
        # Negative correction = too dark = turn right (slow down right motor)
        left_power = base_speed - correction
        right_power = base_speed + correction
        
        # motor power range [-1.0, 1.0]
        left_power = max(-1.0, min(1.0, left_power))
        right_power = max(-1.0, min(1.0, right_power))
        
        # Send power to motors
        left_motor_front.power_command = left_power
        left_motor_back.power_command = left_power
        right_motor_front.power_command = right_power
        right_motor_back.power_command = right_power
        
        # Debug output
        print(f"Lux: {luxSample:6.1f} | Error: {error:6.1f} | "
              f"Correction: {correction:6.2f} | L: {left_power:5.2f} R: {right_power:5.2f}")
        
        # print(f"Encoders - LF: {left_motor_front.position:.2f} LB: {left_motor_back.position:.2f}")
        # print(f"           RF: {right_motor_front.position:.2f} RB: {right_motor_back.position:.2f}")
        
        time.sleep(0.05)  # 20Hz control loop

except KeyboardInterrupt:
    # Stop all motors when program exits
    print("\nStopping motors...")
    left_motor_front.power_command = 0.0
    left_motor_back.power_command = 0.0
    right_motor_front.power_command = 0.0
    right_motor_back.power_command = 0.0
    print("Program terminated.")