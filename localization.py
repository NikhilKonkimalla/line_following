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
#pytime.sleep(3)
#This might need to be a for loop if sensor 
# readings don't change between iterations
whiteCal = [sensor_lux.lux for _ in range (20)]
#meanWhite = sum(whiteCal) / len(whiteCal)
meanWhite = 40
#Calibration on black
print("BEGIN CALIBRATION BLACK")
#pytime.sleep(3)
#This might need to be a for loop if sensor 
# readings don't change between iterations
blackCal = [sensor_lux.lux for _ in range(20)] 
#meanBlack = sum(blackCal) / len(blackCal)
meanBlack = 10

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
#pytime.sleep(5)

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
base_velocity = 7
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
distance_average_period_s = 0.1  # Shortened from 1.0s - faster response to blocks
distance_samples = []
block_distance_threshold = 30.0

# Probabilistic localization parameters (discrete Bayes filter).
# Replace sector_map_bits with the map provided for each trial.
sector_map_bits = [0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0]
sector_length_in = 5.8 #keep eye on this

# Goal sector: set this to the desired destination sector (0-15)
# Set to None to only localize without navigation
goal_sector = 11  # Change this to your desired goal sector (e.g., 0, 5, 12)

# Sensor reliability parameters for Bayesian filter
# Tune these based on your sensor's actual performance
p_detect = 0.9  # P(sensor detects block | block present) - decrease if sensor misses blocks often
p_false = 0.15  # P(sensor detects block | no block) - increase if sensor has many false positives
# Common tuning scenarios:
#   Frequent false positives: p_detect=0.90, p_false=0.15
#   Unreliable sensor: p_detect=0.80, p_false=0.10
#   Conservative/slow convergence: p_detect=0.70, p_false=0.15


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def average_distance_over_period(samples, now, distance, period_s):
    samples.append((now, distance))
    cutoff = now - period_s
    while samples and samples[0][0] < cutoff:
        samples.pop(0)
    return sum(d for _, d in samples) / len(samples)


def normalize_probs(probs):
    total = sum(probs)
    if total <= 0:
        return [1.0 / len(probs)] * len(probs)
    return [p / total for p in probs]


def init_belief_given_block(map_bits):
    weighted = [0.9 if bits == 1 else 0.1 for bits in map_bits]
    return normalize_probs(weighted)


def update_start_sector_beliefs(prior, map_bits, sectors_moved_since_start, block_seen,
                                p_detect, p_false):
    """
    Bayes update:
      posterior(i) ∝ P(z | start=i) * prior(i)

    p_detect: P(sensor says "block" | block truly present)
    p_false:  P(sensor says "block" | block truly absent)
    """
    p_miss = 1.0 - p_detect   # P(no block | block present)
    p_clear = 1.0 - p_false   # P(no block | no block present)
    n = len(map_bits)
    updated = [0.0] * len(prior)
    for start_idx, prior_prob in enumerate(prior):
        expected_block = (map_bits[(start_idx + sectors_moved_since_start) % n] == 1)
        if expected_block:
            likelihood = p_detect if block_seen else p_miss
        else:
            likelihood = p_false if block_seen else p_clear
        updated[start_idx] = prior_prob * likelihood
    return normalize_probs(updated)


print("Starting line following...")

atEnd = False
time = 0.0
# Initialize with uniform belief - don't know where we are yet
beliefs = [1.0 / len(sector_map_bits)] * len(sector_map_bits)
belief_initialized = True  # Start localization immediately
sector_distance_accum = 0.0
total_distance_traveled = 0.0
sectors_moved_since_start = 0

print("Starting probabilistic localization immediately with uniform belief...")

sensor_distance.start_ranging()

try:
    while not atEnd:
        now = pytime.monotonic()
        dt = now - last_loop_t
        raw_distance = sensor_distance.distance
        if raw_distance < 3: raw_distance = 100 
        sensor_distance.clear_interrupt()
        # Use raw distance for both alignment and block detection
        # The 1-second averaging window is too long for fast-moving blocks
        alignment_block = raw_distance < block_distance_threshold
        block = raw_distance < block_distance_threshold
        
        # Keep averaged distance for monitoring/debugging
        distance = average_distance_over_period(
            distance_samples, now, raw_distance, distance_average_period_s
        )

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
        total_distance_traveled += abs(v) * dt
        loop_count += 1

        # Print distance readings in the loop
        # Calculate best sector belief continuously
        best_idx = max(range(len(beliefs)), key=lambda i: beliefs[i])
        best_prob = beliefs[best_idx]
        print(f"raw_distance={raw_distance:.2f} cm, block={block} | Best start sector={best_idx}, p={best_prob:.3f}")
        
        if belief_initialized:
            sector_distance_accum += abs(v) * dt
            while sector_distance_accum >= sector_length_in:
                sector_distance_accum -= sector_length_in
                sectors_moved_since_start += 1
                crossing_block = alignment_block
                beliefs = update_start_sector_beliefs(
                    beliefs, sector_map_bits, sectors_moved_since_start, crossing_block,
                    p_detect, p_false
                )
                best_idx = max(range(len(beliefs)), key=lambda i: beliefs[i])
                print(
                    f">>> SECTOR CROSSING #{sectors_moved_since_start} | "
                    f"Block detected: {crossing_block} | "
                    f"Best start sector={best_idx} p={beliefs[best_idx]:.3f}"
                )
        convergence_threshold = 0.65
        min_sectors_before_stop = len(sector_map_bits)
        if (belief_initialized
                and sectors_moved_since_start >= min_sectors_before_stop
                and max(beliefs) >= convergence_threshold):
            best_idx = max(range(len(beliefs)), key=lambda i: beliefs[i])
            
            # Check if we have a goal sector to navigate to
            if goal_sector is not None:
                # Calculate current sector based on starting sector and movement
                current_sector = (best_idx + sectors_moved_since_start) % len(sector_map_bits)
                
                # Calculate shortest distance to goal (considering circular track)
                forward_distance = (goal_sector - current_sector) % len(sector_map_bits)
                
                # If we're not at the goal, continue moving
                if current_sector != goal_sector:
                    # Only print once when localization first converges
                    if sectors_moved_since_start == min_sectors_before_stop:
                        print(
                            f"Localization converged! Start sector={best_idx}, "
                            f"Current sector={current_sector}, Goal sector={goal_sector}. "
                            f"Distance to goal: {forward_distance} sectors. Continuing..."
                        )
                else:
                    # We've reached the goal!
                    atEnd = True
                    left_motor.velocity_command = 0.0
                    right_motor.velocity_command = 0.0
                    print(
                        f"Goal reached! Start sector={best_idx}, "
                        f"Current sector={current_sector}, Goal sector={goal_sector}. "
                        f"Total distance traveled: {total_distance_traveled:.2f} in. Stopping."
                    )
                    break
            else:
                # No goal sector - stop after localization converges
                atEnd = True
                left_motor.velocity_command = 0.0
                right_motor.velocity_command = 0.0
                print(
                    f"Localization converged! Start sector={best_idx} "
                    f"p={beliefs[best_idx]:.3f}. Stopping."
                )
                break
        time += 0.01
        pytime.sleep(0.01)  # 20Hz control loop

except KeyboardInterrupt:
    # Stop motors when program exits
    print("\nStopping motors...")
    left_motor.velocity_command = 0.0
    right_motor.velocity_command = 0.0
    print("Program terminated.")
