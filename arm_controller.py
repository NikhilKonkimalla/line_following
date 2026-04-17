#!/usr/bin/env python3
"""
arm_controller.py — Motor control for the 2-DOF arm.

Path planning runs on the PC (arm_planner.py). This script connects to it,
sends waypoints, receives joint-space paths, and executes them with PD control.

Usage:
    python arm_controller.py <PLANNER_IP>
"""

import math
import time
import json
import socket
import sys
import tty
import termios
import select
import argparse
import board
from motorgo import Plink, ControlMode

# ── I2C bus init ─────────────────────────────────────────────────────────────
i2c = board.I2C()

# ── Plink / motor setup ─────────────────────────────────────────────────────
plink = Plink()
plink.power_supply_voltage = 9.6

shoulder_motor = plink.channel3
elbow_motor    = plink.channel4

shoulder_motor.motor_voltage_limit = 6.0
elbow_motor.motor_voltage_limit    = 6.0

plink.connect()

shoulder_motor.control_mode = ControlMode.VELOCITY
elbow_motor.control_mode    = ControlMode.VELOCITY

shoulder_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)
elbow_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)

# ── Physical constants ───────────────────────────────────────────────────────
L1 = 4.0
L2 = 4.5

GEAR_RATIO = 1/15
SHOULDER_SIGN = -1
ELBOW_SIGN    =  1

T1_MIN = math.radians(7)
T1_MAX = math.radians(175)
T2_MIN = -math.pi 
T2_MAX = math.pi

# ── Control-loop parameters ──────────────────────────────────────────────────
KP_POS    = 6.0
KD_POS    = 0.5
DT        = 0.005
ANGLE_TOL = math.radians(0.5)
MAX_VEL   = 0.5
JOG_VEL   = 4 

# ── Planner connection ──────────────────────────────────────────────────────
PLANNER_HOST = "192.168.137.1"
PLANNER_PORT = 9090 
_planner_sock = None


def _planner_connect():
    global _planner_sock 
    if _planner_sock is not None:
        return
    _planner_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    _planner_sock.connect((PLANNER_HOST, PLANNER_PORT)) 
    print(f"Connected to planner at {PLANNER_HOST}:{PLANNER_PORT}")


def _planner_request(req: dict) -> dict:
    _planner_connect()
    _planner_sock.sendall((json.dumps(req) + "\n").encode())
    buf = b""
    while b"\n" not in buf:
        data = _planner_sock.recv(16384)
        if not data:
            raise ConnectionError("Planner disconnected")
        buf += data
    line, _ = buf.split(b"\n", 1)
    return json.loads(line.decode())


# ── Velocity-integrated angle tracking ───────────────────────────────────────

_t1_current  = 0.0
_t2_current  = 0.0
_last_update = 0.0


def update_joint_angles():
    global _t1_current, _t2_current, _last_update
    now = time.monotonic()
    dt = now - _last_update
    _last_update = now
    _t1_current += SHOULDER_SIGN * shoulder_motor.velocity * GEAR_RATIO * dt
    _t2_current += ELBOW_SIGN    * elbow_motor.velocity    * GEAR_RATIO * dt


def read_joint_angles():
    update_joint_angles()
    return _t1_current, _t2_current


def read_joint_velocities():
    w1 = SHOULDER_SIGN * shoulder_motor.velocity * GEAR_RATIO
    w2 = ELBOW_SIGN    * elbow_motor.velocity    * GEAR_RATIO
    return w1, w2


def set_joint_velocities(v1, v2):
    shoulder_motor.velocity_command = SHOULDER_SIGN * v1 / GEAR_RATIO
    elbow_motor.velocity_command    = ELBOW_SIGN    * v2 / GEAR_RATIO


def stop():
    shoulder_motor.velocity_command = 0.0
    elbow_motor.velocity_command    = 0.0


# ── Kinematics (local — needed for calibration + display) ───────────────────

def fk(t1, t2):
    x = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    y = L1 * math.sin(t1) + L2 * math.sin(t1 + t2)
    return x, y


def ik(x, y, elbow_up=True):
    r2 = x * x + y * y
    cos_t2 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if abs(cos_t2) > 1.0:
        return None
    sin_t2 = math.sqrt(max(0.0, 1.0 - cos_t2 * cos_t2))
    if elbow_up:
        sin_t2 = -sin_t2
    t2 = math.atan2(sin_t2, cos_t2)
    k1 = L1 + L2 * cos_t2
    k2 = L2 * sin_t2
    t1 = math.atan2(y, x) - math.atan2(k2, k1)
    t1 = t1 % (2 * math.pi)
    if t1 > math.pi:
        t1_alt = t1 - 2 * math.pi
        if T1_MIN <= t1_alt <= T1_MAX:
            t1 = t1_alt
        else:
            return None
    if not (T1_MIN <= t1 <= T1_MAX):
        return None
    if not (T2_MIN <= t2 <= T2_MAX):
        return None
    return t1, t2


# ── Calibration ──────────────────────────────────────────────────────────────

def calibrate():
    global _t1_current, _t2_current, _last_update
    home = ik(6.25, 0.0, elbow_up=True)
    if home is None:
        home = ik(6.25, 0.0, elbow_up=False)
    _t1_current, _t2_current = home
    _last_update = time.monotonic()
    x, y = fk(_t1_current, _t2_current)
    print(f"Calibrated -> t1={math.degrees(_t1_current):.1f} deg  "
          f"t2={math.degrees(_t2_current):.1f} deg  EE=({x:.2f}, {y:.2f}) in")


# ── Joint position control ───────────────────────────────────────────────────

def move_to_angles(goal_t1, goal_t2, timeout=100.0):
    prev_e1 = 0.0
    prev_e2 = 0.0
    t_start = time.monotonic()
    last_t  = t_start

    while True:
        now = time.monotonic()
        if now - t_start > timeout:
            stop()
            print(f"\n  Timeout")
            return False

        dt = now - last_t
        last_t = now

        t1, t2 = read_joint_angles()
        e1 = goal_t1 - t1
        e2 = goal_t2 - t2

        if abs(e1) < ANGLE_TOL and abs(e2) < ANGLE_TOL:
            stop()
            x, y = fk(t1, t2)
            print(f"\n  Reached: EE=({x:.2f},{y:.2f})")
            return True

        de1 = (e1 - prev_e1) / max(dt, 1e-6)
        de2 = (e2 - prev_e2) / max(dt, 1e-6)
        raw_v1 = KP_POS * e1 + KD_POS * de1
        raw_v2 = KP_POS * e2 + KD_POS * de2

        # Scale together so joints move in sync (straight line in joint space)
        max_raw = max(abs(raw_v1), abs(raw_v2))
        if max_raw > MAX_VEL:
            scale = MAX_VEL / max_raw
            v1 = raw_v1 * scale
            v2 = raw_v2 * scale
        else:
            v1 = raw_v1
            v2 = raw_v2

        set_joint_velocities(v1, v2)
        prev_e1 = e1
        prev_e2 = e2

        w1, w2 = read_joint_velocities()
        x, y = fk(t1, t2)
        print(f"  t1={math.degrees(t1):.1f} t2={math.degrees(t2):.1f} "
              f"EE=({x:.2f},{y:.2f}) "
              f"err=({math.degrees(e1):.1f},{math.degrees(e2):.1f})", end="\r")

        time.sleep(DT)


def execute_path(path, x_goal, y_goal, timeout_per_seg=15.0):
    for idx, (wt1, wt2) in enumerate(path[1:], 1):
        wx, wy = fk(wt1, wt2)
        print(f"\nSegment {idx}/{len(path)-1}: EE->({wx:.2f},{wy:.2f})")
        if not move_to_angles(wt1, wt2, timeout=timeout_per_seg):
            return False

    t1, t2 = read_joint_angles()
    x_final, y_final = fk(t1, t2)
    dist = math.sqrt((x_final - x_goal) ** 2 + (y_final - y_goal) ** 2)
    print(f"\n  Reached ({x_final:.2f}, {y_final:.2f})  error: {dist:.3f} in")
    return True


# ── Jog mode ─────────────────────────────────────────────────────────────────

def _get_key(timeout=0.05):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if ready:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def jog_mode():
    print("\n-- JOG MODE --")
    print("  A/D   : shoulder +/-")
    print("  W/S   : elbow    +/-")
    print("  SPACE : stop")
    print("  Q     : exit jog mode")
    print()

    global _t1_current, _t2_current, _last_update
    v1, v2 = 0.0, 0.0

    try:
        while True:
            key = _get_key()
            if key == 'a':
                v1 = JOG_VEL
            elif key == 'd':
                v1 = -JOG_VEL
            elif key == 'w':
                v2 = JOG_VEL
            elif key == 's':
                v2 = -JOG_VEL
            elif key == ' ':
                v1, v2 = 0.0, 0.0
            elif key == 'q':
                stop()
                home = ik(6.25, 0.0, elbow_up=True) or ik(6.25, 0.0, elbow_up=False)
                _t1_current, _t2_current = home
                _last_update = time.monotonic()
                print(f"\n  Jog ended. Arm reset to (6.25, 0).")
                return

            set_joint_velocities(v1, v2)
            update_joint_angles()

            t1, t2 = read_joint_angles()
            x, y = fk(t1, t2)
            print(f"  t1={math.degrees(t1):.1f} t2={math.degrees(t2):.1f}  "
                  f"EE=({x:.2f},{y:.2f})  vel=({v1:+.2f},{v2:+.2f})", end="\r")

            time.sleep(DT)

    except KeyboardInterrupt:
        stop()
        home = ik(6.25, 0.0, elbow_up=True) or ik(6.25, 0.0, elbow_up=False)
        _t1_current, _t2_current = home
        _last_update = time.monotonic()
        print("\n  Jog ended. Arm reset to (6.25, 0).")


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    global PLANNER_HOST, PLANNER_PORT

    parser = argparse.ArgumentParser(description="Arm controller (connects to PC planner)")
    parser.add_argument("planner_ip", nargs="?", default=PLANNER_HOST,
                        help="IP of the PC running arm_planner.py (default: %(default)s)")
    parser.add_argument("--port", type=int, default=PLANNER_PORT)
    args = parser.parse_args()

    PLANNER_HOST = args.planner_ip
    PLANNER_PORT = args.port

    calibrate()

    try:
        _planner_connect()
    except (ConnectionRefusedError, OSError) as e:
        print(f"Cannot reach planner at {PLANNER_HOST}:{PLANNER_PORT}: {e}")
        print("Make sure arm_planner.py is running on your PC.")
        stop()
        return

    print("\nEnter targets as 'x y'. Separate multiple with ';'.")
    print("  Commands: 'home', 'jog', 'q'")
    print("  Examples: '3 4'  or  '3 4 ; -2 5 ; 0 3'")

    while True:
        try:
            cmd = input("\n> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd in ("q", "quit", "exit"):
            break
        if cmd == "jog":
            jog_mode()
            continue

        # Parse waypoints
        if cmd == "home":
            wp_list = [(6.25, 0.0)]
        else:
            wp_strs = cmd.split(";")
            wp_list = []
            bad = False
            for wp_str in wp_strs:
                wp_str = wp_str.strip()
                if not wp_str:
                    continue
                try:
                    parts = wp_str.replace(",", " ").split()
                    wp_list.append((float(parts[0]), float(parts[1])))
                except (ValueError, IndexError):
                    print(f"  Bad waypoint: '{wp_str}'")
                    bad = True
                    break
            if bad or not wp_list:
                print("Usage: x y [; x y ; ...] or 'home' or 'q'")
                continue

        total = len(wp_list)
        start_t1, start_t2 = read_joint_angles()

        # ── Planning phase (runs on PC) ──
        if total == 1:
            gx, gy = wp_list[0]
            print(f"  Planning path to ({gx:.2f}, {gy:.2f})...")
            try:
                resp = _planner_request({
                    "cmd": "plan",
                    "start_t1": start_t1, "start_t2": start_t2,
                    "x_goal": gx, "y_goal": gy,
                })
            except (ConnectionError, OSError) as e:
                print(f"  Planner error: {e}")
                continue

            if "error" in resp:
                print(f"  Error: {resp['error']}")
                continue

            planned = [{"path": [tuple(p) for p in resp["path"]], "x": gx, "y": gy}]
        else:
            print(f"  Planning {total} waypoints...")
            try:
                resp = _planner_request({
                    "cmd": "plan_multi",
                    "start_t1": start_t1, "start_t2": start_t2,
                    "waypoints": wp_list,
                })
            except (ConnectionError, OSError) as e:
                print(f"  Planner error: {e}")
                continue

            if "error" in resp:
                print(f"  Error: {resp['error']}")
                continue

            planned = []
            for entry in resp["paths"]:
                planned.append({
                    "path": [tuple(p) for p in entry["path"]],
                    "x": entry["x"], "y": entry["y"],
                })

        for i, entry in enumerate(planned, 1):
            print(f"  [{i}/{total}] ({entry['x']:.2f}, {entry['y']:.2f}): "
                  f"{len(entry['path'])-1} segment(s)")
        print(f"  All paths planned. Executing...")

        # ── Execution phase (runs locally) ──
        success = True
        for i, entry in enumerate(planned, 1):
            path = entry["path"]
            gx, gy = entry["x"], entry["y"]

            if total > 1:
                print(f"\n{'='*40}")
                print(f"  Waypoint {i}/{total}: ({gx:.2f}, {gy:.2f})")
                print(f"{'='*40}")

            if not execute_path(path, gx, gy):
                print(f"\n  Failed at waypoint {i}/{total}.")
                success = False
                break

            if i < total:
                print(f"\n  Holding for 3 seconds...")
                time.sleep(3)

        if total > 1 and success:
            print(f"\n  All {total} waypoints completed.")

    stop()
    print("Done.")


if __name__ == "__main__":
    main()
