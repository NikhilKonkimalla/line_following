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
left_motor = plink.channel2
right_motor = plink.channel1

# Set motor voltage limits (default is 0)
left_motor.motor_voltage_limit = 6.0
right_motor.motor_voltage_limit = 6.0

# Connect to the board
plink.connect()

# Set control mode to VELOCITY (uses encoders for consistent speed)
left_motor.control_mode = ControlMode.VELOCITY
right_motor.control_mode = ControlMode.VELOCITY

# IMU handle (gyro/accel/mag)
imu = plink.imu

# Set velocity PID gains (P, I, D) - may need tuning for your motors
left_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)
right_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)

# Ensure motors are stopped during calibration
def stop_motors():
    left_motor.velocity_command = 0.0
    right_motor.velocity_command = 0.0

stop_motors()
time.sleep(0.1)

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

#Calculate Threshold and normalized parameters
edgeThreshold = (meanBlack + meanWhite) / 2
# Use max to prevent division by zero if sensor range is too small
sensor_range = max(1.0, abs(meanWhite - meanBlack))
deadzone_norm = 0.08  # Normalized deadzone (8% of range)
print(f"Average White reading: {meanWhite}")
print(f"Average Black reading: {meanBlack}")
print(f"Resulting edge threshold: {edgeThreshold}")
print(f"Sensor range: {sensor_range:.1f} lux")
print(f"Normalized deadzone: {deadzone_norm}")
print("Beginning course traversal in 5 seconds:")
time.sleep(5)

# Ensure motors are still stopped before starting control loop
stop_motors()

#TUNING INSTRUCTIONS
#--------------------------------------------------------------#
# PD CONTROL:
#   p_gain: Proportional gain - responds to current error
#   d_gain: Derivative gain - dampens oscillation, anticipates changes
# 
# VELOCITY:
#   base_velocity: Base forward velocity in rad/s
#   correction_limit: Maximum velocity correction in rad/s
#   min_velocity/max_velocity: Velocity bounds
#   max_delta: Slew rate limit (rad/s per cycle at 20Hz)
#
# MODES:
#   sharp_turn_threshold: Error level to trigger aggressive turn mode
#   lost_line_threshold: Error level to trigger line recovery mode
#--------------------------------------------------------------#

# Velocity Control parameters
# PD Control gains
p_gain = 5.0               # Proportional gain in (rad/s) per normalized error
d_gain = 1.5               # Derivative gain - reduces oscillation and overshooting
base_velocity = 3.5          # Base speed for gentle turns
correction_limit = 1.5     # Max velocity correction in rad/s
min_velocity = 0.1         # Minimum velocity in rad/s
max_velocity = 4         # Maximum velocity cap

# Sharp turn parameters (large errors)
sharp_turn_threshold = 0.25       # Normalized error above this = sharp turn
sharp_turn_p_gain = 8.0          # Higher P gain for aggressive turning
sharp_turn_d_gain = 2.0          # Higher D gain for sharp turns
sharp_turn_base_velocity = 1.0   # Slower base speed during sharp turns
sharp_turn_min_velocity = 0.05   # Allow wheel to almost stop

# Lost line recovery (completely off the line)
lost_line_threshold = 0.3        # Normalized error above this = lost line (left edge only)
recovery_speed = 0.6             # Slow speed while searching
recovery_turn_speed = 0.8        # Speed of wheel that's turning to find line
lost_line_countdown = 0          # Tracks recent "on black" detections
lost_line_countdown_max = 5      # Require recent black before lost-line recovery

# PD control state tracking
prev_error = 0.0                 # Previous error for derivative calculation
dt = 0.05                        # Time step (20Hz control loop)

# Slew rate limiting (prevents jerky changes)
prev_left = 0.0
prev_right = 0.0
max_delta = 0.4                  # rad/s per cycle (increased for faster response)

print("Starting line following...")

try:
    while True:
        # Read sensor directly (no filtering)
        luxSample = sensor.lux
        
        # Raw normalized error (do not modify this)
        raw_error = (luxSample - edgeThreshold) / sensor_range
        
        # Calculate derivative using raw error only
        d_error = (raw_error - prev_error) / dt

        # Update "recently on black" state (right edge following)
        if raw_error < -deadzone_norm:
            lost_line_countdown = lost_line_countdown_max
        else:
            lost_line_countdown = max(0, lost_line_countdown - 1)

        # Apply deadzone - if error is small, go straight (no correction)
        if abs(raw_error) < deadzone_norm:
            # In deadzone: both motors at same velocity for straight line
            left_velocity = base_velocity
            right_velocity = base_velocity
            turn_mode = "STRAIGHT"
            
        elif raw_error > lost_line_threshold:
            # POSSIBLE LOST LINE: Too bright (white)
            if lost_line_countdown > 0:
                # LOST LINE MODE: Drifted OFF line onto white over left edge
                left_velocity = recovery_turn_speed   # Left motor faster
                right_velocity = recovery_speed        # Right motor slower
                turn_mode = "LOST - RIGHT"
            else:
                # Too bright but likely drifted right; handle with normal control
                # Treat like gentle turn to get back to right edge
                adj_error = raw_error - deadzone_norm
                p_term = p_gain * adj_error
                d_term = d_gain * d_error
                correction = p_term + d_term
                correction = max(-correction_limit, min(correction_limit, correction))

                left_velocity = base_velocity - correction
                right_velocity = base_velocity + correction
                left_velocity = min(max_velocity, max(min_velocity, left_velocity))
                right_velocity = min(max_velocity, max(min_velocity, right_velocity))
                turn_mode = "RIGHT EDGE CORRECT"
            
        elif abs(raw_error) > sharp_turn_threshold:
            # SHARP TURN MODE: Large error detected, apply aggressive turning
            # Remove deadzone smoothly to prevent jumps
            if raw_error > 0:
                adj_error = raw_error - deadzone_norm
            else:
                adj_error = raw_error + deadzone_norm
            
            # PD control for sharp turns
            p_term = sharp_turn_p_gain * adj_error
            d_term = sharp_turn_d_gain * d_error
            correction = p_term + d_term
            correction = max(-correction_limit, min(correction_limit, correction))
            
            left_velocity = sharp_turn_base_velocity - correction
            right_velocity = sharp_turn_base_velocity + correction
            
            # Allow wheels to almost stop for tightest turns
            left_velocity = min(max_velocity, max(sharp_turn_min_velocity, left_velocity))
            right_velocity = min(max_velocity, max(sharp_turn_min_velocity, right_velocity))
            turn_mode = "SHARP TURN"
            
        else:
            # GENTLE TURN MODE: Normal differential steering with PD control
            # Remove deadzone smoothly to prevent jumps
            if raw_error > 0:
                adj_error = raw_error - deadzone_norm
            else:
                adj_error = raw_error + deadzone_norm

            # PD control: P term responds to current error, D term dampens oscillation
            p_term = p_gain * adj_error
            d_term = d_gain * d_error
            correction = p_term + d_term
            
            # Over-steering prevention
            correction = max(-correction_limit, min(correction_limit, correction))
            
            # Positive correction = too bright = turn left (slow down left motor)
            # Negative correction = too dark = turn right (slow down right motor)
            left_velocity = base_velocity - correction
            right_velocity = base_velocity + correction
            
            # Clamp velocities to [min_velocity, max_velocity]
            left_velocity = min(max_velocity, max(min_velocity, left_velocity))
            right_velocity = min(max_velocity, max(min_velocity, right_velocity))
            turn_mode = "GENTLE TURN"
        
        # Slew-rate limit: prevent sudden velocity changes
        left_velocity = prev_left + np.clip(left_velocity - prev_left, -max_delta, max_delta)
        right_velocity = prev_right + np.clip(right_velocity - prev_right, -max_delta, max_delta)
        prev_left, prev_right = left_velocity, right_velocity
        
        # Update error history for next derivative calculation
        prev_error = raw_error
        
        # Send velocity commands to motors (rad/s)
        # Left motor reversed, right motor not reversed
        left_motor.velocity_command = -left_velocity
        right_motor.velocity_command = right_velocity
        
        # Debug output with IMU readings
        gyro = imu.gyro
        accel = imu.accel
        mag = imu.mag
        gyro_str = np.array2string(gyro, precision=2, separator=",", suppress_small=True)
        accel_str = np.array2string(accel, precision=2, separator=",", suppress_small=True)
        mag_str = np.array2string(mag, precision=2, separator=",", suppress_small=True)

        print(f"Lux: {luxSample:6.1f} | Err: {raw_error:5.2f} D: {d_error:5.2f} | {turn_mode:12s} | "
              f"L: {left_velocity:5.2f} R: {right_velocity:5.2f} | "
              f"Gyro: {gyro_str} Accel: {accel_str} Mag: {mag_str}")
        
        time.sleep(dt)  # 20Hz control loop

except KeyboardInterrupt:
    # Stop motors when program exits
    print("\nStopping motors...")
    left_motor.velocity_command = 0.0
    right_motor.velocity_command = 0.0
    print("Program terminated.")