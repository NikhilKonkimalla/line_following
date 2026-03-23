#!/usr/bin/env python3
"""Teleop server - runs on the robot (Raspberry Pi).
Receives UDP drive commands from the client and controls drive motors via motorgo/Plink.
Arm is driven by two mirrored 28BYJ-48 stepper motors via ULN2003 driver boards.
"""
import socket
import json
import time
# import threading
import RPi.GPIO as GPIO
from motorgo import Plink, ControlMode

# ----------------------
# STEPPER SETUP (single 28BYJ-48 via ULN2003) — DISABLED
# ----------------------
# # Motor A on GPIO 5/6/13/19 — physical pins 29/31/33/35
# ARM_PINS = [5, 6, 13, 19]  # IN1-IN4 on the ULN2003 board
#
# # Half-step sequence (8 steps) — smoother and more torque than full-step
# HALF_STEP_SEQ = [
#     [1, 0, 0, 0],
#     [1, 1, 0, 0],
#     [0, 1, 0, 0],
#     [0, 1, 1, 0],
#     [0, 0, 1, 0],
#     [0, 0, 1, 1],
#     [0, 0, 0, 1],
#     [1, 0, 0, 1],
# ]
#
# STEP_DELAY    = 0.002   # seconds between steps — tune for speed vs torque
# STEPS_PER_CMD = 16      # half-steps per ARM_LEFT/ARM_RIGHT command received
#
# GPIO.setmode(GPIO.BCM)
# for pin in ARM_PINS:
#     GPIO.setup(pin, GPIO.OUT)
#     GPIO.output(pin, 0)
#
# step_index = 0
#
# def step_motor(index: int, direction: int) -> int:
#     """Fire one half-step. direction: +1 or -1. Returns new index."""
#     index = (index + direction) % len(HALF_STEP_SEQ)
#     for pin, val in zip(ARM_PINS, HALF_STEP_SEQ[index]):
#         GPIO.output(pin, val)
#     return index
#
# def release_motor():
#     """De-energise all coils — reduces heat and eliminates electrical noise."""
#     for pin in ARM_PINS:
#         GPIO.output(pin, 0)
#
# def move_arm(steps: int, direction: int):
#     global step_index
#     for _ in range(steps):
#         step_index = step_motor(step_index, direction)
#         time.sleep(STEP_DELAY)
#     release_motor()
#
# # Run stepper moves on a background thread so the UDP loop stays responsive
# _arm_thread = None
# _arm_lock   = threading.Lock()
#
# def arm_move_async(steps: int, direction: int):
#     """Kick off a non-blocking arm move. Drops command if already moving."""
#     global _arm_thread
#     with _arm_lock:
#         if _arm_thread and _arm_thread.is_alive():
#             return
#         _arm_thread = threading.Thread(
#             target=move_arm, args=(steps, direction), daemon=True
#         )
#         _arm_thread.start()
#
# print(f"Stepper arm initialized on pins {ARM_PINS}")

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

            # --- Arm stepper commands — DISABLED ---
            # elif cmd == "SERVO_LEFT":
            #     arm_move_async(STEPS_PER_CMD, +1)
            # elif cmd == "SERVO_RIGHT":
            #     arm_move_async(STEPS_PER_CMD, -1)
            # elif cmd == "SERVO_CENTER":
            #     release_motor()

            else:
                stop()

        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
        except socket.timeout:
            if time.monotonic() - last_cmd_time >= 0.5:
                stop()

finally:
    stop()
    # release_motor()
    GPIO.cleanup()
    print("\nTeleop stopped.")