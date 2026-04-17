#!/usr/bin/env python3
"""
arm_controller.py  –  Two-link planar arm: IK, velocity-integrated odometry, obstacle avoidance.

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
    elbow_motor    = plink.channel4
    Gear ratio     = 1/5  (motor-shaft rad → joint rad)
    Power supply   = 9.6 V,  voltage limit 6.0 V per motor
    Control mode   = VELOCITY  (Plink firmware PID closes the encoder loop)

Angle tracking:
    Joint angles are tracked by integrating motor.velocity (encoder-derived
    rad/s from firmware) over time. Calibration sets the initial angles from
    IK at a known home pose (EE at 6.25, 0).
"""

import math
import time
import heapq
import sys
import tty
import termios
import select
import board                        # required by all robot files to init hardware
from motorgo import Plink, ControlMode

# ── I2C bus init (present in every working robot file) ────────────────────────
i2c = board.I2C()

# ── Plink / motor setup (identical pattern to NewODO.py) ──────────────────────
plink = Plink() 
plink.power_supply_voltage = 9.6

shoulder_motor = plink.channel3   # joint 1  (θ1)
elbow_motor    = plink.channel4   # joint 2  (θ2)

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
GEAR_RATIO = 1/15 

# Motor direction signs – flip if a joint moves the wrong way
SHOULDER_SIGN =  -1
ELBOW_SIGN    = 1   # mirrors RIGHT_MOTOR_SIGN = -1 in NewODO.py

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
OBS_MARGIN = 0.1   # safety buffer around each obstacle (inches)

# ── Outer position-loop parameters ────────────────────────────────────────────
KP_POS      = 6.0                # position error (rad) → velocity command (rad/s)
KD_POS      = 0.3                # derivative damping
DT          = 0.005              # control-loop timestep (seconds)
ANGLE_TOL   = math.radians(2) # "close enough" to target
MAX_VEL     = 1                # maximum velocity command (rad/s) – same as NewODO drive speed
JOG_VEL     = 4             # joint velocity for manual jog mode (rad/s)

# Path planning: samples per segment for collision checking
N_SAMPLES = 60


# ── Velocity-integrated angle tracking ────────────────────────────────────────
 
_t1_current  = 0.0   # current shoulder angle (rad), updated by integration
_t2_current  = 0.0   # current elbow angle (rad), updated by integration
_last_update = 0.0   # time.monotonic() of last integration step


def update_joint_angles():
    """Integrate motor.velocity to update tracked joint angles."""
    global _t1_current, _t2_current, _last_update
    now = time.monotonic()
    dt = now - _last_update 
    _last_update = now
    w1 = SHOULDER_SIGN * shoulder_motor.velocity * GEAR_RATIO
    w2 = ELBOW_SIGN    * elbow_motor.velocity    * GEAR_RATIO
    _t1_current += w1 * dt
    _t2_current += w2 * dt


def read_joint_angles():
    """Return current (theta1, theta2) from velocity integration."""
    update_joint_angles()
    return _t1_current, _t2_current


def read_joint_velocities(): 
    """Return current (w1, w2) joint velocities (rad/s)."""
    w1 = SHOULDER_SIGN * shoulder_motor.velocity * GEAR_RATIO
    w2 = ELBOW_SIGN    * elbow_motor.velocity    * GEAR_RATIO
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

def calibrate():
    """Assume arm is at home pose with EE at (6.25, 0). Set initial angles from IK."""
    global _t1_current, _t2_current, _last_update

    home = ik(6.25, 0.0, elbow_up=True)
    if home is None:
        home = ik(6.25, 0.0, elbow_up=False)

    _t1_current, _t2_current = home
    _last_update = time.monotonic()

    x, y = fk(_t1_current, _t2_current)
    print(f"Calibrated → θ1={math.degrees(_t1_current):.1f}°  "
          f"θ2={math.degrees(_t2_current):.1f}°  EE=({x:.2f}, {y:.2f}) in")


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
#
# Strategy (from arm_sim.py):
#   1. Try direct line in joint space
#   2. Try one-joint-at-a-time sequential moves
#   3. Fall back to A* grid search over configuration space

def _segment_clear(t1a, t2a, t1b, t2b):
    for i in range(N_SAMPLES + 1):
        a = i / N_SAMPLES
        if config_in_collision(t1a + a * (t1b - t1a), t2a + a * (t2b - t2a)):
            return False
    return True


def _try_sequential_moves(t1a, t2a, t1b, t2b):
    mid = (t1b, t2a)
    if (not config_in_collision(*mid) and
            _segment_clear(t1a, t2a, *mid) and
            _segment_clear(*mid, t1b, t2b)):
        return [(t1a, t2a), mid, (t1b, t2b)]
    mid = (t1a, t2b)
    if (not config_in_collision(*mid) and
            _segment_clear(t1a, t2a, *mid) and
            _segment_clear(*mid, t1b, t2b)):
        return [(t1a, t2a), mid, (t1b, t2b)]
    return None


# ── A* over a discretised configuration space ────────────────────────────────

_GRID_DEG = 5  # degrees per cell — ~36x64 grid

def _angle_to_cell(angle: float, a_min: float) -> int:
    return int(round((angle - a_min) / math.radians(_GRID_DEG)))

def _cell_to_angle(cell: int, a_min: float) -> float:
    return a_min + cell * math.radians(_GRID_DEG)

def _build_collision_grid():
    """Pre-compute which (t1, t2) grid cells are collision-free."""
    n1 = _angle_to_cell(T1_MAX, T1_MIN) + 1
    n2 = _angle_to_cell(T2_MAX, T2_MIN) + 1
    free = set()
    for i1 in range(n1):
        for i2 in range(n2):
            t1 = _cell_to_angle(i1, T1_MIN)
            t2 = _cell_to_angle(i2, T2_MIN)
            if not config_in_collision(t1, t2):
                free.add((i1, i2))
    return free, n1, n2

_cspace_cache = None

def _get_cspace():
    global _cspace_cache
    if _cspace_cache is None:
        _cspace_cache = _build_collision_grid()
    return _cspace_cache


def _astar_grid(start_t1, start_t2, goal_t1, goal_t2):
    """A* search on the configuration-space grid. Returns waypoint list or None."""
    free, n1, n2 = _get_cspace()

    s_i1 = max(0, min(n1 - 1, _angle_to_cell(start_t1, T1_MIN)))
    s_i2 = max(0, min(n2 - 1, _angle_to_cell(start_t2, T2_MIN)))
    g_i1 = max(0, min(n1 - 1, _angle_to_cell(goal_t1, T1_MIN)))
    g_i2 = max(0, min(n2 - 1, _angle_to_cell(goal_t2, T2_MIN)))

    if (g_i1, g_i2) not in free:
        return None

    start = (s_i1, s_i2)
    goal = (g_i1, g_i2)

    if start == goal:
        return [(start_t1, start_t2), (goal_t1, goal_t2)]

    def heuristic(node):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    open_set = [(heuristic(start), 0, start)]
    came_from = {}
    g_score = {start: 0}
    neighbors_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1),
                   (0, 1), (1, -1), (1, 0), (1, 1)]

    while open_set:
        _, cost, current = heapq.heappop(open_set)

        if current == goal:
            cells = [current]
            while current in came_from:
                current = came_from[current]
                cells.append(current)
            cells.reverse()

            # Greedy merge: skip cells as long as segment stays clear
            wp_idx = [0]
            i = 0
            while i < len(cells) - 1:
                best = i + 1
                for j in range(i + 2, len(cells)):
                    a1 = _cell_to_angle(cells[i][0], T1_MIN)
                    a2 = _cell_to_angle(cells[i][1], T2_MIN)
                    b1 = _cell_to_angle(cells[j][0], T1_MIN)
                    b2 = _cell_to_angle(cells[j][1], T2_MIN)
                    if _segment_clear(a1, a2, b1, b2):
                        best = j
                    else:
                        break
                wp_idx.append(best)
                i = best

            result = [(start_t1, start_t2)]
            for idx in wp_idx[1:-1]:
                result.append((_cell_to_angle(cells[idx][0], T1_MIN),
                               _cell_to_angle(cells[idx][1], T2_MIN)))
            result.append((goal_t1, goal_t2))
            return result

        if cost > g_score.get(current, float('inf')):
            continue

        ct1 = _cell_to_angle(current[0], T1_MIN)
        ct2 = _cell_to_angle(current[1], T2_MIN)
        for d1, d2 in neighbors_8:
            nb = (current[0] + d1, current[1] + d2)
            if nb not in free:
                continue
            nt1 = _cell_to_angle(nb[0], T1_MIN)
            nt2 = _cell_to_angle(nb[1], T2_MIN)
            if not _segment_clear(ct1, ct2, nt1, nt2): 
                continue
            step_cost = 1.414 if (d1 != 0 and d2 != 0) else 1.0
            new_cost = cost + step_cost
            if new_cost < g_score.get(nb, float('inf')):
                g_score[nb] = new_cost
                heapq.heappush(open_set, (new_cost + heuristic(nb), new_cost, nb))
                came_from[nb] = current

    return None


def plan_joint_path(start_t1, start_t2, goal_t1, goal_t2):
    """
    Plan a collision-free joint-space path.
    1. Direct line
    2. Sequential (one joint at a time)
    3. A* grid search over configuration space
    """
    if _segment_clear(start_t1, start_t2, goal_t1, goal_t2):
        return [(start_t1, start_t2), (goal_t1, goal_t2)]

    seq = _try_sequential_moves(start_t1, start_t2, goal_t1, goal_t2)
    if seq is not None:
        print(f"  Using sequential move via waypoint")
        return seq

    path = _astar_grid(start_t1, start_t2, goal_t1, goal_t2)
    if path is not None:
        print(f"  A* path: {len(path) - 1} segment(s)")
        return path

    print("  Warning: no collision-free path found")
    return None


# ── Joint position control ─────────────────────────────────────────────────────

def move_to_angles(goal_t1, goal_t2, timeout=15.0):
    """
    Outer position loop: reads joint angles from velocity integration,
    computes PD error, outputs velocity commands to the Plink velocity controller.
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
    if path is None:
        print("  Error: no collision-free path found")
        return False
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


# ── Manual jog mode ───────────────────────────────────────────────────────────

def _get_key(timeout=0.05):
    """Read a single keypress without waiting for Enter. Returns '' on timeout."""
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
    """Manual joint control: A/D = shoulder, W/S = elbow, space = stop, Q = exit."""
    print("\n── JOG MODE ──")
    print("  A/D   : shoulder (joint 1)  +/-")
    print("  W/S   : elbow    (joint 2)  +/-")
    print("  SPACE : stop both joints")
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
                # Reset: arm now considers itself at home (6.25, 0)
                home = ik(6.25, 0.0, elbow_up=True) or ik(6.25, 0.0, elbow_up=False)
                _t1_current, _t2_current = home
                _last_update = time.monotonic()
                print(f"\n  Jog ended. Arm reset to (6.25, 0).")
                return

            set_joint_velocities(v1, v2)
            update_joint_angles()

            t1, t2 = read_joint_angles()
            x, y = fk(t1, t2)
            print(f"  θ1={math.degrees(t1):.1f}°  θ2={math.degrees(t2):.1f}°  "
                  f"EE=({x:.2f},{y:.2f})  vel=({v1:+.2f},{v2:+.2f})", end="\r")
 
            time.sleep(DT)

    except KeyboardInterrupt:
        stop()
        home = ik(6.25, 0.0, elbow_up=True) or ik(6.25, 0.0, elbow_up=False)
        _t1_current, _t2_current = home
        _last_update = time.monotonic()
        print("\n  Jog ended. Arm reset to (6.25, 0).")


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    calibrate()

    print("\nEnter target positions as 'x y' (inches). Separate multiple with ';'.")
    print("  Commands: 'home', 'jog' (manual A/D W/S control), 'q' to quit.")
    print("  Examples: '3 4'  or  '3 4 ; -2 5 ; 0 3'")
    while True:
        try:
            cmd = input("\n> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd in ("q", "quit", "exit"):
            break
        if cmd == "home":
            move_to_xy(6.25, 0.0)
            continue
        if cmd == "jog":
            jog_mode()
            continue

        # Parse waypoints separated by semicolons
        waypoint_strs = cmd.split(";")
        waypoints = []
        parse_error = False
        for wp_str in waypoint_strs:
            wp_str = wp_str.strip()
            if not wp_str:
                continue
            try:
                parts = wp_str.replace(",", " ").split()
                gx, gy = float(parts[0]), float(parts[1])
                waypoints.append((gx, gy))
            except (ValueError, IndexError):
                print(f"  Bad waypoint: '{wp_str}'. Use 'x y' format.")
                parse_error = True
                break

        if parse_error or not waypoints:
            print("Usage: x y [; x y ; ...]  (e.g. '3 4 ; -2 5') or 'home' or 'q'")
            continue

        # Execute waypoints in sequence
        total = len(waypoints)
        success = False
        for i, (gx, gy) in enumerate(waypoints, 1):
            if total > 1:
                print(f"\n{'='*40}")
                print(f"  Waypoint {i}/{total}: ({gx:.2f}, {gy:.2f})")
                print(f"{'='*40}")

            success = move_to_xy(gx, gy)

            if not success:
                if total > 1:
                    print(f"\n  Failed at waypoint {i}/{total}. Remaining skipped.")
                break

            # Wait between waypoints (not after the last one)
            if i < total:
                print(f"\n  Holding at waypoint {i}/{total} for 3 seconds...")
                time.sleep(3)

        if total > 1 and success:
            print(f"\n  All {total} waypoints completed.")

    stop()
    print("Done.")


if __name__ == "__main__":
    main()
