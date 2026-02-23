import time as pytime
import board
import math
import adafruit_bh1750
from motorgo import Plink, ControlMode
import adafruit_vl53l4cx

i2c = board.I2C()

sensor_lux = adafruit_bh1750.BH1750(i2c)

sensor_distance = adafruit_vl53l4cx.VL53L4CX(i2c)

# Initialize Plink board
plink = Plink()
plink.power_supply_voltage = 9.6  # Set to our battery voltage

# Set up motor channels - 1 motor per side
left_motor = plink.channel1
right_motor = plink.channel3

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
pytime.sleep(3)
#This might need to be a for loop if sensor 
# readings don't change between iterations
whiteCal = [sensor_lux.lux for _ in range (20)]
meanWhite = sum(whiteCal) / len(whiteCal)

#Calibration on black
print("BEGIN CALIBRATION BLACK")
pytime.sleep(3)
#This might need to be a for loop if sensor 
# readings don't change between iterations
blackCal = [sensor_lux.lux for _ in range(20)] 
meanBlack = sum(blackCal) / len(blackCal)

#Caluculate Threshold
edgeThreshold = (meanBlack + meanWhite) / 2
# Calculate deadzone as a percentage of the total range
sensor_range = abs(meanWhite - meanBlack)
deadzone = sensor_range * 0.15  # 8% of range for deadzone
print(f"Average White reading: {meanWhite}")
print(f"Average Black reading: {meanBlack}")
print(f"Resulting edge threshold: {edgeThreshold}")
print(f"Deadzone range: +/- {deadzone:.1f} lux")
print("Beginning course traversal in 5 seconds:")
pytime.sleep(5)

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
base_velocity = 6
max_velocity = base_velocity     # Base forward velocity in rad/s
correction_limit = 2.0  # Max velocity correction in rad/s
min_velocity = 0.5      # Minimum velocity in rad/s

# Odometry + slew parameters/state
wheel_radius = 1.15
wheel_base = 5.75
gear_wheel_per_motor = 24.0 / 40.0

pose_x = 0.0
pose_y = 0.0
pose_th = 0.0
loop_count = 0
last_loop_t = pytime.monotonic()


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


print("Starting line following...")

atEnd = False
time = 0.0

try:
    while not atEnd:
        now = pytime.monotonic()
        dt = now - last_loop_t
        last_loop_t = now
        dt = clamp(dt, 1e-3, 0.1)

        luxSample = sensor_lux.lux
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
            correction = clamp(correction, -correction_limit, correction_limit)
            
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
        left_velocity = clamp(left_velocity, -max_velocity, max_velocity)
        right_velocity = clamp(right_velocity, -max_velocity, max_velocity)
        if 0 < abs(left_velocity) < min_velocity:
            left_velocity = min_velocity if left_velocity > 0 else -min_velocity
        if 0 < abs(right_velocity) < min_velocity:
            right_velocity = min_velocity if right_velocity > 0 else -min_velocity
        
        # Send velocity commands to motors (rad/s)
        # Left motor reversed, right motor not reversed
        left_motor.velocity_command = left_velocity
        right_motor.velocity_command = -right_velocity

        # Encoder-based odometry update (starting pose is 0,0,0).
        left_w = left_motor.velocity * gear_wheel_per_motor
        right_w = -right_motor.velocity * gear_wheel_per_motor
        v = wheel_radius * (left_w + right_w) / 2.0
        omega = wheel_radius * (right_w - left_w) / wheel_base
        pose_th = wrap_pi(pose_th + omega * dt)
        pose_x += v * math.cos(pose_th) * dt
        pose_y += v * math.sin(pose_th) * dt
        loop_count += 1
        
        # Debug output with actual velocities from encoders
        if loop_count % 20 == 0:
            print(
                f"Lux: {luxSample:6.1f} | Err: {error:6.1f} | "
                f"L_cmd: {left_velocity:5.2f} R_cmd: {right_velocity:5.2f} | "
                f"x={pose_x:6.2f} in y={pose_y:6.2f} in th={math.degrees(pose_th):6.1f} deg"
            )
        if time>10 and 0<=pose_x<2.5 and 0<=pose_y<2.5:
            atEnd = True
            left_motor.velocity_command = 0.0
            right_motor.velocity_command = 0.0
            print("Reached end of course")
            break
        time += 0.01
        pytime.sleep(0.01)  # 20Hz control loop

except KeyboardInterrupt:
    # Stop motors when program exits
    print("\nStopping motors...")
    left_motor.velocity_command = 0.0
    right_motor.velocity_command = 0.0
    print("Program terminated.")
