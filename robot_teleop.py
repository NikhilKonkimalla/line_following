#!/usr/bin/env python3
"""Teleop server - runs on the robot (Raspberry Pi).
Receives UDP drive commands from the client and controls drive motors via motorgo/Plink.
"""
import socket
import json
import time
import os
import subprocess
from motorgo import Plink, ControlMode

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
print("Controls: WASD=drive, Q/E=pivot, 1-5=speed, SPACE=stop, C=CV toggle")

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
    left_motor1.power_command  = left
    left_motor2.power_command  = left
    right_motor1.power_command = -right
    right_motor2.power_command = -right

def stop():
    set_motors(0.0, 0.0)

# ----------------------
# CV SUBPROCESS
# ----------------------
CV_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cv.py")
cv_proc = None

def start_cv():
    global cv_proc
    if cv_proc is not None and cv_proc.poll() is None:
        print("cv.py is already running.")
        return
    print("Starting cv.py...")
    cv_proc = subprocess.Popen(["python3", CV_SCRIPT])

def stop_cv():
    global cv_proc
    if cv_proc is None or cv_proc.poll() is not None:
        print("cv.py is not running.")
        cv_proc = None
        return
    print("Stopping cv.py...")
    cv_proc.terminate()
    try:
        cv_proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        cv_proc.kill()
    cv_proc = None

# ----------------------
# MAIN LOOP
# ----------------------
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
            elif cmd == "CV_START":
                start_cv()
            elif cmd == "CV_STOP":
                stop_cv()
            elif cmd == "STOP":
                stop()
            else:
                stop()

        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
        except socket.timeout:
            if time.monotonic() - last_cmd_time >= 0.5:
                stop()

finally:
    stop()
    stop_cv()
    print("\nTeleop stopped.")
