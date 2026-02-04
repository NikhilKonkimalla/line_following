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

def odometry(powers, sample_time=3.0, dt=0.01):
    # Wheel/robot geometry (inches)
    wheel_radius = 1.2  # Calibrated: 0.652 * (22/18.94) = 0.757 original was 1.1
    wheel_base = 5.36  # original was 5.25 (best so far is 5.35)

    x, y, theta = 0.0, 0.0, 0.0

    for left_power, right_power in powers:
        left_motor.power_command = left_power
        right_motor.power_command = -right_power

        # Wait for motors to start moving before beginning timing
        
        while True:
            left_w = left_motor.velocity
            right_w = -right_motor.velocity
            if abs(left_w) > 0 or abs(right_w) > 0:
                print("Motors started, beginning odometry...")
                break
            time.sleep(0.01)
        
        start_time = time.monotonic()
        while time.monotonic() - start_time < sample_time:
            # Read encoder velocities (rad/s)
            left_w = left_motor.velocity
            right_w = -right_motor.velocity  # Correct for right motor direction

            # Calculate robot velocity (inches/s) and angular velocity (rad/s)
            v = wheel_radius * (left_w + right_w) / 2.0
            omega = wheel_radius * (right_w - left_w) / wheel_base

            # Simple Euler integration
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += omega * dt

            print(f"Left motor={left_w:.2f} rad/s, Right motor={right_w:.2f} rad/s")

            time.sleep(dt)
        
        # Stop motors between commands
        
        left_motor.power_command = 0
        right_motor.power_command = 0
        count = 0
        while count<100:
            left_w = left_motor.velocity
            right_w = -right_motor.velocity  # Correct for right motor direction

            # Calculate robot velocity (inches/s) and angular velocity (rad/s)
            v = wheel_radius * (left_w + right_w) / 2.0
            omega = wheel_radius * (right_w - left_w) / wheel_base

            # Simple Euler integration
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += omega * dt

            print(f"Left motor={left_w:.2f} rad/s, Right motor={right_w:.2f} rad/s")
            time.sleep(dt)
            count+=1
            
        

    print(f"\nFinal position: x={x:.2f} inches, y={y:.2f} inches, theta={math.degrees(theta):.1f}°")
    return [x, y, theta]


print(odometry([[random.uniform(-1, 1), random.uniform(-1, 1)], [random.uniform(-1, 1), random.uniform(-1, 1)], [random.uniform(-1, 1), random.uniform(-1, 1)]]))
#print(odometry([[0.5, -0.5]]))
x1, y1 = random.uniform(-1, 1)
x2, y2 = random.uniform(-1, 1)
x3, y3 = random.uniform(-1, 1)

print(odometry([[x1, y1], [x2, y2], [x3, y3]]))
print("l1: " + x1)
print("r1: " + y1)
print("l2: " + x2)
print("r2: " + y2)
print("l3: " + x3)
print("r3: " + y3)