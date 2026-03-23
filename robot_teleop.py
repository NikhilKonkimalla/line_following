#!/usr/bin/env python3
"""Teleop server - runs on the robot (Raspberry Pi).
Receives UDP drive commands from the client and controls motors via motorgo/Plink.
Also controls two mirrored SG90 servos on GPIO 18 (Pin 12) and GPIO 12 (Pin 32).
"""
import socket
import json
import time
import RPi.GPIO as GPIO
from motorgo import Plink, ControlMode

# ----------------------
# SERVO SETUP (two mirrored SG90s)
# ----------------------
# Update these pins once you know which GPIO you're wiring to:
SERVO_PIN_A = 18        # left servo  — GPIO 18, physical Pin 12
SERVO_PIN_B = 12        # right servo — GPIO 12, physical Pin 32

SERVO_FREQ = 50         # Hz — standard for hobby servos
SERVO_MIN_DC = 2.5      # duty cycle for 0°
SERVO_MAX_DC = 12.5     # duty cycle for 180°
SERVO_STEP = 2          # degrees per command (client sends ~33/sec while held)

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN_A, GPIO.OUT)
GPIO.setup(SERVO_PIN_B, GPIO.OUT)

servo_a = GPIO.PWM(SERVO_PIN_A, SERVO_FREQ)
servo_b = GPIO.PWM(SERVO_PIN_B, SERVO_FREQ)
servo_a.start(7.5)  # center both at 90°
servo_b.start(7.5)
current_angle = 90.0    # single shared angle — servos move as one arm

def angle_to_dc(angle: float) -> float:
    """Convert angle (0-180) to duty cycle."""
    angle = max(0.0, min(180.0, angle))
    return SERVO_MIN_DC + (angle / 180.0) * (SERVO_MAX_DC - SERVO_MIN_DC)

def set_arm(angle: float):
    """Move both servos together. Because they are mirrored, servo B gets
    the inverse angle so both push/pull the arm in the same direction.
    PWM is cut after a short pulse so the idle signal doesn't bleed into motors."""
    global current_angle
    current_angle = max(0.0, min(180.0, angle))
    mirrored_angle = 180.0 - current_angle
    servo_a.ChangeDutyCycle(angle_to_dc(current_angle))
    servo_b.ChangeDutyCycle(angle_to_dc(mirrored_angle))
    time.sleep(0.3)         # give servo time to reach position
    servo_a.ChangeDutyCycle(0)  # cut signal — servo holds mechanically
    servo_b.ChangeDutyCycle(0)

def arm_left():
    set_arm(current_angle - SERVO_STEP)

def arm_right():
    set_arm(current_angle + SERVO_STEP)

def arm_center():
    set_arm(90.0)

print(f"Arm servos initialized on GPIO {SERVO_PIN_A} + GPIO {SERVO_PIN_B}, centered at 90°")

# ----------------------
# MOTOR SETUP (no encoders needed)
# ----------------------
plink = Plink()
plink.power_supply_voltage = 9.6

right_motor1 = plink.channel1
right_motor2 = plink.channel2
left_motor1 = plink.channel3
left_motor2 = plink.channel4

right_motor1.motor_voltage_limit = 6.0
right_motor2.motor_voltage_limit = 6.0
left_motor1.motor_voltage_limit = 6.0
left_motor2.motor_voltage_limit = 6.0

plink.connect()

right_motor1.control_mode = ControlMode.POWER
right_motor2.control_mode = ControlMode.POWER
left_motor1.control_mode = ControlMode.POWER
left_motor2.control_mode = ControlMode.POWER

# Quick motor test
print("Testing motors...")
right_motor1.power_command = 1.0
right_motor2.power_command = 1.0
left_motor1.power_command = 1.0
left_motor2.power_command = 1.0
time.sleep(0.3)
right_motor1.power_command = 0.0
right_motor2.power_command = 0.0
left_motor1.power_command = 0.0
left_motor2.power_command = 0.0
print("Motor test done.")

# ----------------------
# NETWORK SETUP
# ----------------------
TELEOP_PORT = 7124
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", TELEOP_PORT))
sock.settimeout(0.5)  # timeout so we can stop motors if no commands received

print(f"Teleop server listening on UDP port {TELEOP_PORT}")
print("Controls: WASD=drive, Q/E=pivot, [/]=servo, \\=servo center, 1-5=speed, SPACE=stop")

# ----------------------
# DRIVE LOGIC
# ----------------------
BASE_POWER = 3.0  # default power level
MAX_POWER = 6.0   # must match motor_voltage_limit
current_power = BASE_POWER

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def set_motors(left: float, right: float):
    left = clamp(left, -MAX_POWER, MAX_POWER)
    right = clamp(right, -MAX_POWER, MAX_POWER)
    left_motor1.power_command = -left
    left_motor2.power_command = -left
    # right motors are mounted opposite
    right_motor1.power_command = right
    right_motor2.power_command = right

def stop():
    set_motors(0.0, 0.0)

last_cmd_time = time.monotonic()

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg = json.loads(data.decode())
            cmd = msg.get("cmd", "STOP")
            power = clamp(msg.get("power", current_power), 0, MAX_POWER)
            current_power = power
            last_cmd_time = time.monotonic()

            # --- Drive commands ---
            if cmd == "FWD":
                set_motors(power, power)
            elif cmd == "BACK":
                set_motors(-power, -power)
            elif cmd == "LEFT":
                set_motors(-power, power)
            elif cmd == "RIGHT":
                set_motors(power, -power)
            elif cmd == "FWD_LEFT":
                set_motors(0, power)
            elif cmd == "FWD_RIGHT":
                set_motors(power, 0)
            elif cmd == "BACK_LEFT":
                set_motors(-power * 0.1, -power)
            elif cmd == "BACK_RIGHT":
                set_motors(-power, -power * 0.1)
            elif cmd == "STOP":
                stop()

            # --- Arm servo commands ---
            elif cmd == "SERVO_LEFT":
                arm_left()
            elif cmd == "SERVO_RIGHT":
                arm_right()
            elif cmd == "SERVO_CENTER":
                arm_center()

            else:
                stop()

        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
        except socket.timeout:
            if time.monotonic() - last_cmd_time >= 0.5:
                stop()

finally:
    stop()
    servo_a.stop()
    servo_b.stop()
    GPIO.cleanup()
    print("\nTeleop stopped.")