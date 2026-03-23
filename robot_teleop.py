#!/usr/bin/env python3
"""Teleop server - runs on the robot (Raspberry Pi).
Receives UDP drive commands from the client and controls drive motors via motorgo/Plink.
Arm is driven by two mirrored 28BYJ-48 stepper motors via ULN2003 driver boards.
"""
import socket
import json
import time
import threading
import RPi.GPIO as GPIO
from motorgo import Plink, ControlMode

# ----------------------
# STEPPER SETUP (28BYJ-48 via ULN2003)
# ----------------------
# Change these pin numbers to match however you wire up your ULN2003 boards.
# Motor A = left stepper, Motor B = right stepper (mirrored — runs reversed).
MOTOR_A_PINS = [5, 6, 13, 19]   # IN1-IN4 for left  — GPIO, physical pins 29,31,33,35
MOTOR_B_PINS = [10, 9, 11, 25]  # IN1-IN4 for right — GPIO, physical pins 19,21,23,22

<<<<<<< HEAD
SERVO_FREQ = 50         # Hz — standard for hobby servos
SERVO_MIN_DC = 2.5      # duty cycle for 0°
SERVO_MAX_DC = 12.5   # duty cycle for 180° (2.5ms / 20ms * 100)
SERVO_STEP = 1          # degrees per command (client sends ~33/sec while held)
=======
# Half-step sequence (8 steps) — smoother and more torque than full-step
HALF_STEP_SEQ = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1],
]

STEP_DELAY    = 0.002   # seconds between steps — tune for speed vs torque
STEPS_PER_CMD = 16      # half-steps per ARM_LEFT/ARM_RIGHT command received
>>>>>>> 81f1c6d20a254fff01409dc0e02a5b1dcf10783e

GPIO.setmode(GPIO.BCM)
for pin in MOTOR_A_PINS + MOTOR_B_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

<<<<<<< HEAD
current_angle = 90.0    # single shared angle — servos move as one arm
=======
step_index_a = 0
step_index_b = 0
>>>>>>> 81f1c6d20a254fff01409dc0e02a5b1dcf10783e

def step_motor(pins: list, index: int, direction: int) -> int:
    """Fire one half-step. direction: +1 or -1. Returns new index."""
    index = (index + direction) % len(HALF_STEP_SEQ)
    for pin, val in zip(pins, HALF_STEP_SEQ[index]):
        GPIO.output(pin, val)
    return index

<<<<<<< HEAD
def set_arm(angle: float):
    """Move both servos together. Because they are mirrored, servo B gets
    the inverse angle so both push/pull the arm in the same direction."""
    global current_angle
    current_angle = max(0.0, min(180.0, angle))
    mirrored_angle = 180.0 - current_angle
    servo_a.ChangeDutyCycle(angle_to_dc(current_angle))
    servo_b.ChangeDutyCycle(angle_to_dc(mirrored_angle))
    time.sleep(0.02)
    servo_a.ChangeDutyCycle(0)
    servo_b.ChangeDutyCycle(0)
=======
def release_motor(pins: list):
    """De-energise all coils — reduces heat and eliminates electrical noise."""
    for pin in pins:
        GPIO.output(pin, 0)
>>>>>>> 81f1c6d20a254fff01409dc0e02a5b1dcf10783e

def move_arm(steps: int, dir_a: int):
    """Move both steppers together. Motor B runs opposite because it is mirrored."""
    global step_index_a, step_index_b
    dir_b = -dir_a
    for _ in range(steps):
        step_index_a = step_motor(MOTOR_A_PINS, step_index_a, dir_a)
        step_index_b = step_motor(MOTOR_B_PINS, step_index_b, dir_b)
        time.sleep(STEP_DELAY)
    release_motor(MOTOR_A_PINS)
    release_motor(MOTOR_B_PINS)

# Run stepper moves on a background thread so the UDP loop stays responsive
_arm_thread = None
_arm_lock   = threading.Lock()

def arm_move_async(steps: int, direction: int):
    """Kick off a non-blocking arm move. Drops command if already moving."""
    global _arm_thread
    with _arm_lock:
        if _arm_thread and _arm_thread.is_alive():
            return
        _arm_thread = threading.Thread(
            target=move_arm, args=(steps, direction), daemon=True
        )
        _arm_thread.start()

<<<<<<< HEAD
servo_a = GPIO.PWM(SERVO_PIN_A, SERVO_FREQ)
servo_b = GPIO.PWM(SERVO_PIN_B, SERVO_FREQ)
servo_a.start(angle_to_dc(90))
servo_b.start(angle_to_dc(90))
time.sleep(0.5)
servo_a.ChangeDutyCycle(0)
servo_b.ChangeDutyCycle(0)
print(f"Arm servos initialized on GPIO {SERVO_PIN_A} + GPIO {SERVO_PIN_B}, centered at 90°")
=======
print("Stepper arm initialized.")
print(f"  Motor A pins (left) : {MOTOR_A_PINS}")
print(f"  Motor B pins (right): {MOTOR_B_PINS}")
>>>>>>> 81f1c6d20a254fff01409dc0e02a5b1dcf10783e

# ----------------------
# DRIVE MOTOR SETUP (Plink)
# ----------------------
plink = Plink()
plink.power_supply_voltage = 9.6

right_motor1 = plink.channel1
right_motor2 = plink.channel2
left_motor1  = plink.channel3
left_motor2  = plink.channel4

right_motor1.motor_voltage_limit = 6.0
right_motor2.motor_voltage_limit = 6.0
left_motor1.motor_voltage_limit  = 6.0
left_motor2.motor_voltage_limit  = 6.0

plink.connect()

right_motor1.control_mode = ControlMode.POWER
right_motor2.control_mode = ControlMode.POWER
left_motor1.control_mode  = ControlMode.POWER
left_motor2.control_mode  = ControlMode.POWER

print("Testing drive motors...")
for m in [right_motor1, right_motor2, left_motor1, left_motor2]:
    m.power_command = 1.0
time.sleep(0.3)
for m in [right_motor1, right_motor2, left_motor1, left_motor2]:
    m.power_command = 0.0
print("Drive motor test done.")

# ----------------------
# NETWORK SETUP
# ----------------------
TELEOP_PORT = 7124
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", TELEOP_PORT))
sock.settimeout(0.5)

print(f"Teleop server listening on UDP port {TELEOP_PORT}")
print("Controls: WASD=drive, Q/E=pivot, [/]=arm, 1-5=speed, SPACE=stop")

# ----------------------
# DRIVE LOGIC
# ----------------------
MAX_POWER     = 6.0
current_power = 3.0

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def set_motors(left: float, right: float):
    left  = clamp(left,  -MAX_POWER, MAX_POWER)
    right = clamp(right, -MAX_POWER, MAX_POWER)
    left_motor1.power_command  = -left
    left_motor2.power_command  = -left
    right_motor1.power_command =  right
    right_motor2.power_command =  right

def stop():
    set_motors(0.0, 0.0)

last_cmd_time = time.monotonic()

try:
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            msg   = json.loads(data.decode())
            cmd   = msg.get("cmd", "STOP")
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

            # --- Arm stepper commands ---
            elif cmd == "SERVO_LEFT":
                arm_move_async(STEPS_PER_CMD, +1)
            elif cmd == "SERVO_RIGHT":
                arm_move_async(STEPS_PER_CMD, -1)
            elif cmd == "SERVO_CENTER":
                # De-energise coils immediately
                release_motor(MOTOR_A_PINS)
                release_motor(MOTOR_B_PINS)

            else:
                stop()

        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
        except socket.timeout:
            if time.monotonic() - last_cmd_time >= 0.5:
                stop()

finally:
    stop()
    release_motor(MOTOR_A_PINS)
    release_motor(MOTOR_B_PINS)
    GPIO.cleanup()
    print("\nTeleop stopped.")