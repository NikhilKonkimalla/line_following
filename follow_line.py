import numpy as np
import time
import board
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
# Calculate deadzone as a percentage of the total range
sensor_range = abs(meanWhite - meanBlack)
deadzone = sensor_range * 0.08  # 8% of range for deadzone
print(f"Average White reading: {meanWhite}")
print(f"Average Black reading: {meanBlack}")
print(f"Resulting edge threshold: {edgeThreshold}")
print(f"Deadzone range: +/- {deadzone:.1f} lux")
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
gain = 0.2             # Proportional gain (start low, increase if sluggish)
base_speed = 0.8      # Base forward speed (0.0 to 1.0)
correction_limit = 0.3  # Max correction to prevent wild turns
min_speed = 0.15       # Minimum speed to keep wheels always moving

print("Starting line following...")

try:
    while True:
        luxSample = sensor.lux
        error = luxSample - edgeThreshold
        
        # Apply deadzone - if error is small, go straight (no correction)
        if abs(error) < deadzone:
            # In deadzone: both motors at same speed for straight line
            correction = 0.0
            left_power = base_speed
            right_power = base_speed
        else:
            # Outside deadzone: apply steering correction
            # Reduce error by deadzone amount to smooth transition
            if error > 0:
                error = error - deadzone
            else:
                error = error + deadzone
            correction = gain * error
            
            #over-steering prevention
            correction = max(-correction_limit, min(correction_limit, correction))
            
            # Positive correction = too bright = turn left (slow down left motor)
            # Negative correction = too dark = turn right (slow down right motor)
            left_power = base_speed - correction
            right_power = base_speed + correction
        
        # motor power range [min_speed, 1.0] - always moving, never backwards
        left_power = max(min_speed, min(1.0, left_power))
        right_power = max(min_speed, min(1.0, right_power))
        
        # Send power to motors (reversed with negative sign)
        left_motor.power_command = -left_power
        right_motor.power_command = -right_power
        
        # Debug output
        print(f"Lux: {luxSample:6.1f} | Error: {error:6.1f} | "
              f"Correction: {correction:6.2f} | L: {left_power:5.2f} R: {right_power:5.2f}")
        
        # Optional: Read encoder positions for debugging
        # print(f"Encoders - L: {left_motor.position:.2f} R: {right_motor.position:.2f}")
        
        time.sleep(0.05)  # 20Hz control loop

except KeyboardInterrupt:
    # Stop motors when program exits
    print("\nStopping motors...")
    left_motor.power_command = 0.0
    right_motor.power_command = 0.0
    print("Program terminated.")