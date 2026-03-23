#!/usr/bin/env python3
"""Teleop server - runs on the robot (Raspberry Pi).
Receives UDP drive commands from the client and controls motors via motorgo/Plink.
"""
import socket
import json
import time
from motorgo import Plink, ControlMode

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
print("Controls: WASD=drive, Q/E=pivot, 1-5=speed, SPACE=stop")

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
            else:
                stop()

        except (json.JSONDecodeError, UnicodeDecodeError):
            # Malformed packet - ignore it
            pass 
        except socket.timeout:
            # Safety: stop if no commands for 0.5s
            if time.monotonic() - last_cmd_time >= 0.5:
                stop()

finally:
    stop()
    print("\nTeleop stopped.")
