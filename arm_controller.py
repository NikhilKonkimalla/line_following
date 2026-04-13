#!/usr/bin/env python3
"""
arm_controller.py  –  Two-link planar arm: IK, encoder odometry, obstacle avoidance.

Arm geometry (inches):
    L1 = 4.0   shoulder → elbow
    L2 = 4.5   elbow    → end-effector

Joint limits (from spec):
    theta1 : 0  → pi  rad  (0° = +x, 180° = -x)
    theta2 : -pi → pi rad  (relative to link-1)

Playing field obstacle rectangles (inches, measured from arm base):
    Left  block:  x ∈ [-4, -2],  y ∈ [4, 6]
    Right block:  x ∈ [ 1,  3],  y ∈ [5, 7]
    (edit OBSTACLES below if the measured values differ)

Hardware (same as NewODO.py / odometry.py):
    shoulder_motor = plink.channel3
    elbow_motor    = plink.channel2
    Gear ratio     = 24/40  (motor-shaft rad → joint rad)
    Power supply   = 9.6 V,  voltage limit 6.0 V per motor
    Control mode   = VELOCITY  (Plink firmware PID closes the encoder loop)

Angle tracking:
    Uses motor.position (cumulative encoder radians from firmware, same field
    that plink.py populates via update_position() each SPI tick).
    At calibration we record the raw encoder value at a known pose; after that:
        joint_angle = (motor.position - enc_at_cal) * GEAR_RATIO + angle_at_cal

Outer control loop:
    Reads current joint angles from motor.position, computes position error,
    outputs a velocity command → sent to the Plink velocity controller.
    Mirrors the read_wheel_vel() + update_pose() pattern of NewODO.py.
"""

import math
import time
import board                        # required by all robot files to init hardware
from motorgo import Plink, ControlMode

# ── I2C bus init (present in every working robot file) ────────────────────────
i2c = board.I2C()

# ── Plink / motor setup (identical pattern to NewODO.py) ──────────────────────
plink = Plink()
plink.power_supply_voltage = 9.6

shoulder_motor = plink.channel3   # joint 1  (θ1)
elbow_motor    = plink.channel2   # joint 2  (θ2)

shoulder_motor.motor_voltage_limit = 6.0
elbow_motor.motor_voltage_limit    = 6.0

plink.connect()

# VELOCITY mode: Plink firmware PID uses encoder feedback to hit commanded rad/s
shoulder_motor.control_mode = ControlMode.VELOCITY
elbow_motor.control_mode    = ControlMode.VELOCITY

# Velocity PID gains – same starting values as NewODO.py; tune if needed
shoulder_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)
elbow_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)

# ── Physical constants ─────────────────────────────────────────────────────────
L1 = 4.0          # inches, link 1 (shoulder → elbow)
L2 = 4.5          # inches, link 2 (elbow → end-effector)

# Gear ratio: motor-shaft rad → joint rad  (24/40 from odometry.py / NewODO.py)
GEAR_RATIO = 24.0 / 40.0

# Motor direction signs – flip if a joint moves the wrong way
SHOULDER_SIGN =  1
ELBOW_SIGN    = -1   # mirrors RIGHT_MOTOR_SIGN = -1 in NewODO.py

# Joint limits (radians)
T1_MIN = 0.0
T1_MAX = math.pi
T2_MIN = -math.pi
T2_MAX = math.pi

# ── Obstacle definitions (inches, world frame, origin = arm base) ──────────────
# (x_min, x_max, y_min, y_max) – estimated from playing-field image
OBSTACLES = [
    (-4.0, -2.0, 4.0, 6.0),   # left block
    ( 1.0,  3.0, 5.0, 7.0),   # right block
]
OBS_MARGIN = 0.3   # safety buffer around each obstacle (inches)

# ── Outer position-loop parameters ────────────────────────────────────────────
KP_POS      = 6.0                # position error (rad) → velocity command (rad/s)
KD_POS      = 0.3                # derivative damping
DT          = 0.005              # control-loop timestep (seconds)
ANGLE_TOL   = math.radians(2.0) # "close enough" to target
MAX_VEL     = 8.0                # maximum velocity command (rad/s) – same as NewODO drive speed

# Path planning: samples per segment for collision checking
N_SAMPLES = 60


# ── Encoder reading (mirrors read_wheel_vel() in NewODO.py) ───────────────────

# Calibration offsets – set by calibrate()
_s_enc_cal   = 0.0    # shoulder encoder value at calibration
_e_enc_cal   = 0.0    # elbow    encoder value at calibration
_s_angle_cal = math.pi / 2   # shoulder joint angle at calibration (rad)
_e_angle_cal = 0.0           # elbow    joint angle at calibration (rad)


def read_joint_angles():
    """
    Return current (theta1, theta2) from motor.position encoder values.

    motor.position is the cumulative encoder position in radians as reported
    by the Plink firmware (populated by update_position() each SPI tick in plink.py).
    We apply the gear ratio and direction sign exactly like NewODO.py does for velocity:
        wl = LEFT_MOTOR_SIGN * (left_motor.velocity * GEAR_WHEEL_PER_MOTOR)
    """
    t1 = _s_angle_cal + (shoulder_motor.position - _s_enc_cal) * GEAR_RATIO * SHOULDER_SIGN
    t2 = _e_angle_cal + (elbow_motor.position    - _e_enc_cal) * GEAR_RATIO * ELBOW_SIGN
    return t1, t2


def read_joint_velocities():
    """
    Return current (w1, w2) joint velocities (rad/s).
    Same formula as NewODO.py's read_wheel_vel():
        wl = LEFT_MOTOR_SIGN * (left_motor.velocity * GEAR_WHEEL_PER_MOTOR)
    """
    w1 = SHOULDER_SIGN * (shoulder_motor.velocity * GEAR_RATIO)
    w2 = ELBOW_SIGN    * (elbow_motor.velocity    * GEAR_RATIO)
    return w1, w2


def set_joint_velocities(v1, v2):
    """
    Send velocity commands (rad/s) to both joints.
    Mirrors set_wheel_velocity() in NewODO.py – converts joint rad/s to
    motor rad/s and applies direction sign before commanding the Plink.
    """
    motor_s = SHOULDER_SIGN * v1 / GEAR_RATIO   # joint rad/s → motor rad/s
    motor_e = ELBOW_SIGN    * v2 / GEAR_RATIO
    shoulder_motor.velocity_command = motor_s
    elbow_motor.velocity_command    = motor_e


def stop():
    shoulder_motor.velocity_command = 0.0
    elbow_motor.velocity_command    = 0.0


# ── Calibration ────────────────────────────────────────────────────────────────

def calibrate(home_t1=math.pi / 2, home_t2=0.0):
    """
    Record encoder zero-point at a known physical pose.
    Physically position the arm at:
        θ1 = home_t1  (default 90° – arm pointing straight up)
        θ2 = home_t2  (default  0° – links aligned)
    then press Enter.
    """
    global _s_enc_cal, _e_enc_cal, _s_angle_cal, _e_angle_cal

    input(f"\nPlace arm at home pose "
          f"(θ1={math.degrees(home_t1):.0f}°, "
          f"θ2={math.degrees(home_t2):.0f}°) then press Enter... ")

    time.sleep(0.3)   # let vibration settle before reading encoder

    _s_enc_cal   = shoulder_motor.position
    _e_enc_cal   = elbow_motor.position
    _s_angle_cal = home_t1
    _e_angle_cal = home_t2

    t1, t2 = read_joint_angles()
    x, y = fk(t1, t2)
    print(f"Calibrated → θ1={math.degrees(t1):.1f}°  "
          f"θ2={math.degrees(t2):.1f}°  EE=({x:.2f}, {y:.2f}) in")


# ── Kinematics ─────────────────────────────────────────────────────────────────

def fk(t1, t2):
    """Forward kinematics → (x, y) of end-effector (inches)."""
    x = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    y = L1 * math.sin(t1) + L2 * math.sin(t1 + t2)
    return x, y


def ik(x, y, elbow_up=True):
    """
    Closed-form inverse kinematics.
    elbow_up=True → theta2 < 0 (standard above-the-line solution).
    Returns (theta1, theta2) in radians, or None if unreachable / outside limits.
    """
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

    # Normalise t1 into [0, π]
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


# ── Collision checking ─────────────────────────────────────────────────────────

def _arm_sample_points(t1, t2, n_per_link=25):
    """Dense (x,y) samples along both arm links for collision checking."""
    pts = []
    ex  = L1 * math.cos(t1)
    ey  = L1 * math.sin(t1)
    eex, eey = fk(t1, t2)

    for i in range(n_per_link + 1):
        a = i / n_per_link
        pts.append((a * ex, a * ey))

    for i in range(n_per_link + 1):
        a = i / n_per_link
        pts.append((ex + a * (eex - ex), ey + a * (eey - ey)))

    return pts


def _pt_in_obstacle(x, y):
    for (x0, x1, y0, y1) in OBSTACLES:
        if (x0 - OBS_MARGIN <= x <= x1 + OBS_MARGIN and
                y0 - OBS_MARGIN <= y <= y1 + OBS_MARGIN):
            return True
    return False


def config_in_collision(t1, t2):
    """True if any point on either arm link is inside an obstacle."""
    for (x, y) in _arm_sample_points(t1, t2):
        if _pt_in_obstacle(x, y):
            return True
    return False


# ── Joint-space path planning ──────────────────────────────────────────────────

def _segment_clear(t1a, t2a, t1b, t2b):
    for i in range(N_SAMPLES + 1):
        a = i / N_SAMPLES
        if config_in_collision(t1a + a * (t1b - t1a), t2a + a * (t2b - t2a)):
            return False
    return True


def plan_joint_path(start_t1, start_t2, goal_t1, goal_t2):
    """
    Return ordered (t1, t2) waypoints from start to goal.
    1. Try direct straight line in joint space.
    2. If blocked, route via canonical safe intermediate poses.
    """
    if _segment_clear(start_t1, start_t2, goal_t1, goal_t2):
        return [(start_t1, start_t2), (goal_t1, goal_t2)]

    safe_poses = [
        (math.pi / 2,        0.0),
        (math.pi / 2,  math.pi / 4),
        (math.pi / 2, -math.pi / 4),
        (math.pi / 4,        0.0),
        (3 * math.pi / 4,    0.0),
        (math.pi / 3,  math.pi / 6),
        (2 * math.pi / 3, -math.pi / 6),
    ]

    for mid_t1, mid_t2 in safe_poses:
        if (not config_in_collision(mid_t1, mid_t2) and
                _segment_clear(start_t1, start_t2, mid_t1, mid_t2) and
                _segment_clear(mid_t1, mid_t2, goal_t1, goal_t2)):
            print(f"  Routing via waypoint "
                  f"θ1={math.degrees(mid_t1):.0f}° θ2={math.degrees(mid_t2):.0f}°")
            return [(start_t1, start_t2), (mid_t1, mid_t2), (goal_t1, goal_t2)]

    print("  Warning: no clear path found – using direct route")
    return [(start_t1, start_t2), (goal_t1, goal_t2)]


# ── Joint position control ─────────────────────────────────────────────────────

def move_to_angles(goal_t1, goal_t2, timeout=15.0):
    """
    Outer position loop: reads joint angles from encoder (motor.position),
    computes PD error, outputs velocity commands to the Plink velocity controller.

    Mirrors NewODO.py's pattern:
        update_pose(dt) reads encoders → pose error → set_wheel_velocity()
    """
    prev_e1 = 0.0
    prev_e2 = 0.0
    t_start  = time.monotonic()
    last_t   = t_start

    while True:
        now = time.monotonic()
        if now - t_start > timeout:
            stop()
            print(f"\n  Timeout – θ1={math.degrees(read_joint_angles()[0]):.1f}° "
                  f"θ2={math.degrees(read_joint_angles()[1]):.1f}°")
            return False

        dt = now - last_t
        last_t = now

        t1, t2 = read_joint_angles()
        e1 = goal_t1 - t1
        e2 = goal_t2 - t2

        if abs(e1) < ANGLE_TOL and abs(e2) < ANGLE_TOL:
            stop()
            x, y = fk(t1, t2)
            print(f"\n  Reached: θ1={math.degrees(t1):.1f}°  "
                  f"θ2={math.degrees(t2):.1f}°  EE=({x:.2f},{y:.2f})")
            return True

        # PD → velocity command
        de1 = (e1 - prev_e1) / max(dt, 1e-6)
        de2 = (e2 - prev_e2) / max(dt, 1e-6)
        v1 = max(-MAX_VEL, min(MAX_VEL, KP_POS * e1 + KD_POS * de1))
        v2 = max(-MAX_VEL, min(MAX_VEL, KP_POS * e2 + KD_POS * de2))

        set_joint_velocities(v1, v2)
        prev_e1 = e1
        prev_e2 = e2

        w1, w2 = read_joint_velocities()
        x, y = fk(t1, t2)
        print(f"  w1={w1:.2f} w2={w2:.2f} rad/s | "
              f"θ1={math.degrees(t1):.1f}° θ2={math.degrees(t2):.1f}° | "
              f"EE=({x:.2f},{y:.2f}) | "
              f"err=({math.degrees(e1):.1f}°,{math.degrees(e2):.1f}°)", end="\r")

        time.sleep(DT)


# ── Cartesian move with IK + obstacle avoidance ────────────────────────────────

def move_to_xy(x_goal, y_goal, elbow_up=True, timeout_per_seg=15.0):
    """
    Move end-effector to (x_goal, y_goal) inches (world frame, origin = arm base).
    Solves IK, checks obstacles, plans a joint-space path, executes segment by segment.
    """
    print(f"\nTarget EE: ({x_goal:.2f}, {y_goal:.2f}) in")

    # IK
    sol = ik(x_goal, y_goal, elbow_up=elbow_up)
    if sol is None:
        sol = ik(x_goal, y_goal, elbow_up=not elbow_up)
        if sol is None:
            print("  Error: target is out of reach")
            return False

    goal_t1, goal_t2 = sol
    print(f"  IK: θ1={math.degrees(goal_t1):.1f}°  θ2={math.degrees(goal_t2):.1f}°")

    if config_in_collision(goal_t1, goal_t2):
        alt = ik(x_goal, y_goal, elbow_up=not elbow_up)
        if alt is not None and not config_in_collision(*alt):
            goal_t1, goal_t2 = alt
            print(f"  Switched elbow solution: "
                  f"θ1={math.degrees(goal_t1):.1f}°  θ2={math.degrees(goal_t2):.1f}°")
        else:
            print("  Error: goal is inside an obstacle")
            return False

    start_t1, start_t2 = read_joint_angles()
    print(f"  Start: θ1={math.degrees(start_t1):.1f}°  θ2={math.degrees(start_t2):.1f}°")

    path = plan_joint_path(start_t1, start_t2, goal_t1, goal_t2)
    print(f"  Path: {len(path) - 1} segment(s)")

    for idx, (wt1, wt2) in enumerate(path[1:], 1):
        wx, wy = fk(wt1, wt2)
        print(f"\nSegment {idx}/{len(path)-1}: "
              f"θ1={math.degrees(wt1):.1f}° θ2={math.degrees(wt2):.1f}°  "
              f"EE→({wx:.2f},{wy:.2f})")
        if not move_to_angles(wt1, wt2, timeout=timeout_per_seg):
            return False

    t1, t2 = read_joint_angles()
    x_final, y_final = fk(t1, t2)
    dist = math.sqrt((x_final - x_goal) ** 2 + (y_final - y_goal) ** 2)
    print(f"\nFinal EE: ({x_final:.3f}, {y_final:.3f})  "
          f"goal: ({x_goal:.3f}, {y_goal:.3f})  error: {dist:.3f} in")
    return True


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    # Calibrate: place arm at home pose (straight up, θ1=90°, θ2=0°) then Enter
    calibrate()

    # Set your target here (inches, world frame, origin = arm base)
    targets = [
        (3.0, 3.0),   # example – replace with your actual goal
    ]

    for (gx, gy) in targets:
        if not move_to_xy(gx, gy):
            print(f"  Failed to reach ({gx}, {gy}), stopping.")
            break
        time.sleep(0.5)

    stop()
    print("Done.")


if __name__ == "__main__":
    main()
