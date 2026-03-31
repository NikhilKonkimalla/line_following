#!/usr/bin/env python3
"""Teleop server - runs on the robot (Raspberry Pi).
Receives UDP drive commands from the client and controls drive motors via motorgo/Plink.
"""
import socket
import json
import time
import os
import subprocess
import threading
import RPi.GPIO as GPIO
from motorgo import Plink, ControlMode

# ----------------------
# DRIVE MOTOR SETUP (Plink)
# ----------------------
plink = Plink() 
plink.power_supply_voltage = 9.6

left_motor  = plink.channel1
right_motor = plink.channel3

left_motor.motor_voltage_limit  = 6.0
right_motor.motor_voltage_limit = 6.0

plink.connect()

left_motor.control_mode  = ControlMode.POWER
right_motor.control_mode = ControlMode.POWER

print("Testing drive motors...")
left_motor.power_command  = 1.0
right_motor.power_command = 1.0
time.sleep(0.3)
left_motor.power_command  = 0.0
right_motor.power_command = 0.0
print("Drive motor test done.")

# ----------------------
# STEPPER SETUP (ULN2003 / 28BYJ-48)
# IN1=GPIO17, IN2=GPIO18, IN3=GPIO27, IN4=GPIO22
# ----------------------
STEPPER_PINS  = [17, 18, 27, 22]
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
STEP_DELAY      = 0.002   # seconds per half-step (~500 steps/sec)
STEPPER_TIMEOUT = 0.15    # auto-stop if no command received for 150 ms

GPIO.setmode(GPIO.BCM)
for _pin in STEPPER_PINS:
    GPIO.setup(_pin, GPIO.OUT)
    GPIO.output(_pin, 0)

stepper_direction = 0     # 0=stop, 1=forward, -1=backward
stepper_step_idx  = 0
last_stepper_cmd  = 0.0

def _stepper_thread():
    global stepper_direction, stepper_step_idx
    while _stepper_running:
        # Auto-stop when the client stops sending commands (key released)
        if stepper_direction != 0 and (time.monotonic() - last_stepper_cmd) > STEPPER_TIMEOUT:
            stepper_direction = 0

        if stepper_direction != 0:
            stepper_step_idx = (stepper_step_idx + stepper_direction) % 8
            for pin, val in zip(STEPPER_PINS, HALF_STEP_SEQ[stepper_step_idx]):
                GPIO.output(pin, val)
            time.sleep(STEP_DELAY)
        else:
            time.sleep(0.005)   # idle poll

_stepper_running = True
_stepper_th = threading.Thread(target=_stepper_thread, daemon=True)
_stepper_th.start()
print("Stepper thread started.")

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
    left_motor.power_command  =  clamp(left,  -MAX_POWER, MAX_POWER)
    right_motor.power_command = -clamp(right, -MAX_POWER, MAX_POWER)

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
            elif cmd == "SERVO_LEFT":
                stepper_direction = 1
                last_stepper_cmd  = time.monotonic()
            elif cmd == "SERVO_RIGHT":
                stepper_direction = -1
                last_stepper_cmd  = time.monotonic()
            elif cmd == "SERVO_CENTER":
                stepper_direction = 0
                for pin in STEPPER_PINS:
                    GPIO.output(pin, 0)   # deenergize coils

        except (json.JSONDecodeError, UnicodeDecodeError):
            pass
        except socket.timeout:
            if time.monotonic() - last_cmd_time >= 0.5:
                stop()

finally:
    stop()
    stop_cv()
    _stepper_running = False
    _stepper_th.join(timeout=1)
    for pin in STEPPER_PINS:
        GPIO.output(pin, 0)
    GPIO.cleanup()
    print("\nTeleop stopped.")
