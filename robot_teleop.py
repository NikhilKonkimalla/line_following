#!/usr/bin/env python3
"""Teleop server - runs on the robot (Raspberry Pi).
Receives UDP drive commands from the client and controls drive motors via motorgo/Plink.
"""
import socket
import json
import time
import threading
import board
import adafruit_vl53l4cd as adafruit_vl53l4cx
import RPi.GPIO as GPIO
from motorgo import Plink, ControlMode

# ----------------------
# TOF SENSOR SETUP (must init I2C before Plink grabs the bus)
# ----------------------
i2c = board.I2C()
vl53 = adafruit_vl53l4cx.VL53L4CD(i2c) 
vl53.start_ranging()
print("ToF sensor ready.")

def read_tof_cm() -> float:
    """Read ToF distance in cm. Returns 999.0 if no valid reading."""
    if vl53.data_ready:
        dist = vl53.distance
        vl53.clear_interrupt()
        if dist is not None and dist > 0: 
            return dist
    return 999.0

# ----------------------
# DRIVE MOTOR SETUP (Plink)
# ----------------------
plink = Plink()
plink.power_supply_voltage = 9.6

left_motor1  = plink.channel1
left_motor2  = plink.channel2
right_motor1 = plink.channel3
right_motor2 = plink.channel4

for m in [left_motor1, left_motor2, right_motor1, right_motor2]:
    m.motor_voltage_limit = 6.0

plink.connect()

for m in [left_motor1, left_motor2, right_motor1, right_motor2]:
    m.control_mode = ControlMode.POWER

print("Testing drive motors...")
for m in [left_motor1, left_motor2, right_motor1, right_motor2]:
    m.power_command = 1.0
time.sleep(0.3)
for m in [left_motor1, left_motor2, right_motor1, right_motor2]:
    m.power_command = 0.0
print("Drive motor test done.")

# ----------------------
# STEPPER SETUP (ULN2003 / 28BYJ-48)
# IN1=GPIO17, IN2=GPIO18, IN3=GPIO27, IN4=GPIO26
# ----------------------
STEPPER_PINS  = [17, 18, 27, 26]
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
STEP_DELAY_UP   = 0.003   # seconds per half-step going up
STEP_DELAY_DOWN = 0.008   # slower going down (gravity assist causes overshoot)
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
            stepper_step_idx = (stepper_step_idx + stepper_direction) % len(HALF_STEP_SEQ)
            for pin, val in zip(STEPPER_PINS, HALF_STEP_SEQ[stepper_step_idx]):
                GPIO.output(pin, val)
            delay = STEP_DELAY_DOWN if stepper_direction == -1 else STEP_DELAY_UP
            time.sleep(delay)
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
    left  = clamp(left,  -MAX_POWER, MAX_POWER)
    right = clamp(right, -MAX_POWER, MAX_POWER)
    left_motor1.power_command  = -left
    left_motor2.power_command  = -left
    right_motor1.power_command = right
    right_motor2.power_command = right

def stop():
    set_motors(0.0, 0.0)


# ----------------------
# IMU HELPERS
# ----------------------
imu = plink.imu

def get_yaw_rate() -> float:
    """Return yaw rate in deg/s from IMU gyro (z-axis)."""
    return imu.gyro[2] 

# ----------------------
# CV THREAD MANAGEMENT
# ----------------------
cv_stop_event = threading.Event()
cv_thread = None
client_addr_for_status = None

status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_status(status: str):
    if client_addr_for_status is None:
        return
    msg = json.dumps({"status": status}).encode()
    try:
        status_sock.sendto(msg, (client_addr_for_status, 7125))
    except OSError:
        pass

def start_cv_thread(target, *args):
    global cv_thread
    stop_cv_thread()
    cv_stop_event.clear()
    cv_thread = threading.Thread(target=target, args=args, daemon=True)
    cv_thread.start()

def stop_cv_thread():
    global cv_thread
    if cv_thread is not None and cv_thread.is_alive():
        cv_stop_event.set()
        cv_thread.join(timeout=3)
    cv_thread = None

# ----------------------
# CV TASKS
# ----------------------
pose_th = 0.0

def cv_turn_task(deg: float):
    """Turn in place by deg degrees using IMU gyro integration."""
    TURN_POWER    = 3.0
    MIN_POWER     = 1.5
    SLOW_DEG      = 25.0
    BRAKE_POWER   = 2.0
    BRAKE_TIME    = 0.08
    TIMEOUT       = 12.0

    sign = 1.0 if deg >= 0 else -1.0
    target = abs(deg)

    try:
        turned = 0.0
        t0 = prev_t = time.monotonic()

        while not cv_stop_event.is_set():
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now
            if dt <= 0:
                dt = 0.001

            if now - t0 > TIMEOUT:
                print("[TURN] timeout")
                break

            yaw_rate = get_yaw_rate()  # deg/s
            turned += yaw_rate * dt

            remaining = target - abs(turned)
            if remaining <= 0:
                break

            if remaining < SLOW_DEG:
                power = MIN_POWER + (TURN_POWER - MIN_POWER) * (remaining / SLOW_DEG)
            else:
                power = TURN_POWER

            # positive deg = left turn: left motors back, right motors fwd
            set_motors(-sign * power, sign * power)
            time.sleep(0.005)

        # brake pulse
        set_motors(sign * BRAKE_POWER, -sign * BRAKE_POWER)
        time.sleep(BRAKE_TIME)
        stop()

        print(f"[TURN] Done. Turned {turned:.1f} deg")
        if cv_stop_event.is_set():
            send_status("TURN_ERROR")
        else:
            send_status("TURN_DONE")
    except Exception as e:
        print(f"[TURN] Error: {e}")
        stop()
        send_status("TURN_ERROR")

def cv_drive_task():
    """Drive forward with IMU heading-hold until stopped."""
    DRIVE_POWER = 3.0
    K_P         = 0.05
    TIMEOUT     = 30.0

    try:
        heading = 0.0
        t0 = prev_t = time.monotonic()

        while not cv_stop_event.is_set():
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now
            if dt <= 0:
                dt = 0.001

            if now - t0 > TIMEOUT:
                break

            yaw_rate = get_yaw_rate()
            heading += yaw_rate * dt

            correction = K_P * heading
            set_motors(DRIVE_POWER - correction, DRIVE_POWER + correction)
            time.sleep(0.005)

        stop()
        send_status("DRIVE_STOPPED")
    except Exception as e:
        print(f"[DRIVE] Error: {e}")
        stop()
        send_status("DRIVE_ERROR")

def cv_drive_tof_task(distance_cm: float = 15.24):
    """Drive forward with IMU heading-hold, stop after ToF measures distance_cm traveled."""
    DRIVE_POWER = 3.0
    K_P         = 0.05
    TIMEOUT     = 30.0

    try:
        initial_dist = read_tof_cm()
        for _ in range(5):
            time.sleep(0.05)
            d = read_tof_cm()
            if d < initial_dist:
                initial_dist = d
        print(f"[DRIVE_TOF] Initial: {initial_dist:.1f} cm, target: {distance_cm:.1f} cm")

        heading = 0.0
        t0 = prev_t = time.monotonic()

        while not cv_stop_event.is_set():
            now = time.monotonic()
            dt = now - prev_t
            prev_t = now
            if dt <= 0:
                dt = 0.001

            if now - t0 > TIMEOUT:
                print("[DRIVE_TOF] Timeout")
                break

            yaw_rate = get_yaw_rate()
            heading += yaw_rate * dt

            correction = K_P * heading
            set_motors(DRIVE_POWER - correction, DRIVE_POWER + correction)

            current_dist = read_tof_cm()
            traveled = initial_dist - current_dist
            if traveled >= distance_cm:
                print(f"[DRIVE_TOF] Reached target ({traveled:.1f} cm)")
                break

            time.sleep(0.005)

        stop()
        send_status("DRIVE_DONE")
    except Exception as e:
        print(f"[DRIVE_TOF] Error: {e}")
        stop()
        send_status("DRIVE_ERROR")

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
            elif cmd == "TURN":
                client_addr_for_status = addr[0]
                deg = float(msg.get("deg", 90))
                start_cv_thread(cv_turn_task, deg)
            elif cmd == "DRIVE_HEADING":
                client_addr_for_status = addr[0]
                start_cv_thread(cv_drive_task)
            elif cmd == "DRIVE_TOF":
                client_addr_for_status = addr[0]
                dist = float(msg.get("distance_cm", 15.24))
                start_cv_thread(cv_drive_tof_task, dist)
            elif cmd == "DRIVE_STOP":
                stop_cv_thread()
            elif cmd == "CV_STOP":
                stop_cv_thread()
                stop()
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
                if cv_thread is None or not cv_thread.is_alive():
                    stop()

finally:
    stop_cv_thread()
    stop()
    _stepper_running = False
    _stepper_th.join(timeout=1)
    for pin in STEPPER_PINS:
        GPIO.output(pin, 0)
    GPIO.cleanup()
    print("\nTeleop stopped.")
