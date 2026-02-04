import time
import board
import math
import adafruit_bh1750
from motorgo import Plink, ControlMode
import random

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

# Set control mode to POWER (i.e. simple)
left_motor.control_mode = ControlMode.POWER
right_motor.control_mode = ControlMode.POWER

def odometry(powers, sample_time=3.0, dt=0.01, velocity_stop_threshold=0.05,
             coast_stable_count=5, max_coast_time=3.0):
    # Wheel/robot geometry (inches)
    wheel_radius = 1.15  # Calibrated: 0.652 * (22/18.94) = 0.757 original was 1.1
    wheel_base = 6  # original was 5.25 (best so far is 5.35)

    # Coast-down: integrate until both motors below velocity_stop_threshold (rad/s)
    # for coast_stable_count consecutive samples, or until max_coast_time (s) is reached.

    x, y, theta = 0.0, 0.0, 0.0

    for left_power, right_power in powers:
        # Gear reverses both wheels: negate both so logical forward (positive) => both motors negative => physical forward
        left_motor.power_command = left_power
        right_motor.power_command = -right_power

        # Wait for motors to start moving before beginning timing
        
        while True:
            left_w = left_motor.velocity * 24/40
            right_w = -right_motor.velocity * 24/40
            if abs(left_w) > 0 or abs(right_w) > 0:
                print("Motors started, beginning odometry...")
                break
        
        start_time = time.monotonic()
        while time.monotonic() - start_time < sample_time:
            # Read encoder velocities (rad/s) — sign flipped for gear reversal
            left_w = left_motor.velocity * 24/40
            right_w = -right_motor.velocity * 24/40

            # Calculate robot velocity (inches/s) and angular velocity (rad/s)
            v = wheel_radius * (left_w + right_w) / 2.0
            omega = wheel_radius * (left_w - right_w) / wheel_base

            # Simple Euler integration
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += omega * dt

            print(f"Left motor={left_w:.2f} rad/s, Right motor={right_w:.2f} rad/s")

            time.sleep(dt)
        
        # Stop motors and coast until both below velocity_stop_threshold
        left_motor.power_command = 0
        right_motor.power_command = 0
        stable_count = 0
        coast_start = time.monotonic()
        while stable_count < coast_stable_count: # and (time.monotonic() - coast_start) < max_coast_time
            left_w = left_motor.velocity * 24/40
            right_w = -right_motor.velocity * 24/40

            v = wheel_radius * (left_w + right_w) / 2.0
            omega = wheel_radius * (left_w - right_w) / wheel_base
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += omega * dt

            if abs(left_w) < velocity_stop_threshold and abs(right_w) < velocity_stop_threshold:
                stable_count += 1
            else:
                stable_count = 0

            print(f"Left motor={left_w:.2f} rad/s, Right motor={right_w:.2f} rad/s")
            time.sleep(dt)



    print(f"\nFinal position: x={x:.2f} inches, y={y:.2f} inches, theta={math.degrees(theta):.1f}°")
    return [x, y, theta]


#print(odometry([[random.uniform(-1, 1), random.uniform(-1, 1)], [random.uniform(-1, 1), random.uniform(-1, 1)], [random.uniform(-1, 1), random.uniform(-1, 1)]]))
print(odometry([[0.7, 0.9], [0.55, -0.55], [1, 1]]))

