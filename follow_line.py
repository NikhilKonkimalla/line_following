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
left_motor = plink.channel3
right_motor = plink.channel2

# Set motor voltage limits (default is 0)
left_motor.motor_voltage_limit = 6.0
right_motor.motor_voltage_limit = 6.0

# Connect to the board
plink.connect()

# Set control mode to VELOCITY (uses encoders for consistent speed)
left_motor.control_mode = ControlMode.VELOCITY
right_motor.control_mode = ControlMode.VELOCITY

# Set velocity PID gains (P, I, D) - may need tuning for your motors
left_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)
right_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)

# Ensure motors are stopped during calibration
left_motor.velocity_command = 0.0
right_motor.velocity_command = 0.0

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
deadzone = sensor_range * 0.08 # 5 or 7 percent for challenge and 0.08 for base board
print(f"Average White reading: {meanWhite}")
print(f"Average Black reading: {meanBlack}") 
print(f"Resulting edge threshold: {edgeThreshold}")
print(f"Deadzone range: +/- {deadzone:.1f} lux")
print("Beginning course traversal in 5 seconds:")
time.sleep(5)

#TUNING INSTRUCTIONS
#--------------------------------------------------------------#
# gain: Controls how aggressively robot corrects
# base_velocity: Base forward velocity in rad/s
# correction_limit: Maximum velocity correction to prevent over-steering
# min_velocity: Minimum velocity to keep wheels always moving
# Velocity PID gains: Tune if motors don't track velocity well
#--------------------------------------------------------------#

# Velocity Control parameters
gain = 0.3              # Proportional gain (increased for sharper turns)
base_velocity = 7.5 #base board is 7.5, 5.5 challenge is 7, 5
max_velocity = 5.5    # Base forward velocity in rad/s
correction_limit = 2.0  # Max velocity correction in rad/s
min_velocity = 0.5      # Minimum velocity in rad/s

print("Starting line following...")

try:
    while True:
        luxSample = sensor.lux
        error = luxSample - edgeThreshold

        # Apply deadzone - if error is small, go straight (no correction)
        if abs(error) < deadzone:
            # In deadzone: both motors at same velocity for straight line
            left_velocity = base_velocity
            right_velocity = base_velocity
        else:
            # Outside deadzone: apply steering correction
            # Reduce error by deadzone amount to smooth transition
            if error > 0:
                error = error - deadzone
            else:
                error = error + deadzone
            correction = gain * error
            
            # over-steering prevention
            correction = max(-correction_limit, min(correction_limit, correction))
            
            # In-place turn: motors spin in opposite directions
            turn_speed = max(min_velocity, base_velocity + abs(correction))
            turn_speed = min(max_velocity, turn_speed)
            if error > 0:
                # Too bright => turn left (left reverse, right forward)
                left_velocity = -turn_speed
                right_velocity = turn_speed
            else:
                # Too dark => turn right (left forward, right reverse)
                left_velocity = turn_speed
                right_velocity = -turn_speed
        
        # Clamp signed velocities to min/max magnitude
        left_velocity = max(-max_velocity, min(max_velocity, left_velocity))
        right_velocity = max(-max_velocity, min(max_velocity, right_velocity))
        if 0 < abs(left_velocity) < min_velocity:
            left_velocity = min_velocity if left_velocity > 0 else -min_velocity
        if 0 < abs(right_velocity) < min_velocity:
            right_velocity = min_velocity if right_velocity > 0 else -min_velocity
        
        # Send velocity commands to motors (rad/s)
        # Gear reverses both wheels: negate both so logical forward => both motors negative => physical forward
        left_motor.velocity_command = left_velocity 
        right_motor.velocity_command = -right_velocity

        # Debug output: L_act/R_act in logical convention (match L_cmd/R_cmd)
        print(f"Lux: {luxSample:6.1f} | Error: {error:6.1f} | "
              f"L_cmd: {left_velocity:5.2f} L_act: {-left_motor.velocity:5.2f} | "
              f"R_cmd: {right_velocity:5.2f} R_act: {-right_motor.velocity:5.2f}")
        
        time.sleep(0.01)  # 20Hz control loop

except KeyboardInterrupt:
    # Stop motors when program exits
    print("\nStopping motors...")
    left_motor.velocity_command = 0.0
    right_motor.velocity_command = 0.0
    print("Program terminated.")