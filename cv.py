#!/usr/bin/env python3
"""
cv.py — Autonomous ball-knock task.

Phases:
  1. SCAN  – Capture camera frames and detect the blue arrow direction (LEFT or RIGHT).
  2. TURN  – Rotate 90° toward the indicated ball using encoder odometry.
  3. DRIVE – Drive forward with heading-hold until SPACE is pressed.
"""

import time
import math
import board
import numpy as np
import cv2
from motorgo import Plink, ControlMode

# ─────────────────────────────────────────────────────────────────────────────
# Hardware init  (mirrors NewODO.py / odometry.py conventions)
# ─────────────────────────────────────────────────────────────────────────────
i2c   = board.I2C()
plink = Plink()
plink.power_supply_voltage = 9.6

left_motor  = plink.channel1
right_motor = plink.channel3

left_motor.motor_voltage_limit  = 6.0
right_motor.motor_voltage_limit = 6.0

plink.connect()

left_motor.control_mode  = ControlMode.VELOCITY
right_motor.control_mode = ControlMode.VELOCITY

left_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)
right_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)

# Motor direction signs (same as NewODO.py)
LEFT_MOTOR_SIGN  =  1
RIGHT_MOTOR_SIGN = -1

# Robot geometry (inches)
WHEEL_RADIUS_IN      = 1.15
WHEEL_BASE_IN        = 6.0
GEAR_WHEEL_PER_MOTOR = 24 / 40   # wheel speed = motor speed * this
GEAR_MOTOR_PER_WHEEL = 40 / 24   # motor command = wheel speed * this

# ─────────────────────────────────────────────────────────────────────────────
# Odometry pose  (global, updated by update_pose())
# ─────────────────────────────────────────────────────────────────────────────
POSE_X  = 0.0   # inches
POSE_Y  = 0.0   # inches
POSE_TH = 0.0   # radians  (+CCW, -CW)

# ─────────────────────────────────────────────────────────────────────────────
# Motor helpers
# ─────────────────────────────────────────────────────────────────────────────
def set_wheel_velocity(wl: float, wr: float) -> None:
    """Command wheel angular velocities (rad/s) in the wheel frame."""
    left_motor.velocity_command  = LEFT_MOTOR_SIGN  * wl * GEAR_MOTOR_PER_WHEEL
    right_motor.velocity_command = RIGHT_MOTOR_SIGN * wr * GEAR_MOTOR_PER_WHEEL


def stop() -> None:
    left_motor.velocity_command  = 0.0
    right_motor.velocity_command = 0.0


def read_wheel_vel():
    """Return (wl, wr) in rad/s in the wheel frame (same sign convention as set_wheel_velocity)."""
    wl = LEFT_MOTOR_SIGN  * (left_motor.velocity  * GEAR_WHEEL_PER_MOTOR)
    wr = RIGHT_MOTOR_SIGN * (right_motor.velocity * GEAR_WHEEL_PER_MOTOR)
    return wl, wr


def _wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def update_pose(dt: float) -> None:
    """Integrate encoder velocities into POSE_X / POSE_Y / POSE_TH."""
    global POSE_X, POSE_Y, POSE_TH
    wl, wr  = read_wheel_vel()
    v       = WHEEL_RADIUS_IN * (wl + wr) / 2.0
    omega   = WHEEL_RADIUS_IN * (wl - wr) / WHEEL_BASE_IN
    POSE_X  += v * math.cos(POSE_TH) * dt
    POSE_Y  += v * math.sin(POSE_TH) * dt
    POSE_TH  = _wrap_pi(POSE_TH + omega * dt)

# ─────────────────────────────────────────────────────────────────────────────
# Turn in-place  (from NewODO.py)
# ─────────────────────────────────────────────────────────────────────────────
def turn_angle(
    deg:             float,
    max_turn_speed:  float = 5.0,
    min_turn_speed:  float = 1.5,
    slow_down_deg:   float = 25.0,
    timeout_s:       float = 12.0,
    use_brake_pulse: bool  = True,
    brake_speed:     float = 2.0,
    brake_time:      float = 0.08,
) -> None:
    """Turn in place by deg degrees (+CCW, -CW) using encoder odometry."""
    if deg == 0:
        return

    target   = math.radians(deg)
    start_th = POSE_TH
    sign     = 1.0 if deg > 0 else -1.0
    t0 = last_t = time.monotonic()

    while True:
        now    = time.monotonic()
        dt     = now - last_t
        last_t = now

        update_pose(dt)

        turned    = _wrap_pi(POSE_TH - start_th)
        remaining = math.degrees(max(0.0, abs(target) - abs(turned)))

        # Ramp down near target to reduce overshoot
        if remaining < slow_down_deg:
            frac  = remaining / slow_down_deg
            speed = min_turn_speed + frac * (max_turn_speed - min_turn_speed)
        else:
            speed = max_turn_speed

        set_wheel_velocity(-sign * speed, +sign * speed)

        print(
            f"[TURN] th={math.degrees(POSE_TH):+.1f} deg  "
            f"turned={math.degrees(turned):+.1f}/{deg:+.0f} deg  "
            f"remaining={remaining:.1f} deg  speed={speed:.2f}"
        )

        if abs(turned) >= abs(target):
            break

        if now - t0 > timeout_s:
            stop()
            raise RuntimeError("Turn timed out (check motor direction signs).")

        time.sleep(0.002)

    # Short brake pulse to kill momentum
    if use_brake_pulse:
        set_wheel_velocity(+sign * brake_speed, -sign * brake_speed)
        time.sleep(brake_time)

    stop()
    time.sleep(0.05)
    print(f"[TURN] Done.  Final heading: {math.degrees(POSE_TH):.1f} deg")

# ─────────────────────────────────────────────────────────────────────────────
# Arrow detection  (from arrow_direction.py — identical HSV logic)
# ─────────────────────────────────────────────────────────────────────────────
LOWER_BLUE      = np.array([ 90,  30,  10])
UPPER_BLUE      = np.array([135, 255, 150])
MIN_BLUE_PIXELS = 50


def detect_arrow_in_frame(frame) -> str:
    """Return 'LEFT', 'RIGHT', 'UNCERTAIN', or 'NO ARROW'."""
    hsv         = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask        = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
    blue_pixels = np.where(mask > 0)

    if len(blue_pixels[1]) < MIN_BLUE_PIXELS:
        return "NO ARROW"

    xs          = blue_pixels[1]
    centroid_x  = (xs.min() + xs.max()) / 2.0
    left_count  = np.sum(xs <  centroid_x)
    right_count = np.sum(xs >= centroid_x)

    if left_count > right_count:
        return "LEFT"
    elif right_count > left_count:
        return "RIGHT"
    return "UNCERTAIN"

# ─────────────────────────────────────────────────────────────────────────────
# Phase 1 – Scan for arrow
# ─────────────────────────────────────────────────────────────────────────────
CONFIRM_FRAMES = 8   # consecutive consistent detections required


def scan_for_arrow(cap) -> str:
    """
    Stream camera frames until CONFIRM_FRAMES consecutive frames all agree
    on LEFT or RIGHT.  Returns the confirmed direction string.
    Press ESC to abort.
    """
    print("=== PHASE 1: Scanning for arrow (ESC to abort) ===")
    consecutive = 0
    last_dir    = None

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        direction = detect_arrow_in_frame(frame)

        if direction in ("LEFT", "RIGHT"):
            if direction == last_dir:
                consecutive += 1
            else:
                last_dir    = direction
                consecutive = 1
        else:
            consecutive = 0
            last_dir    = None

        # HUD overlay
        confirmed = consecutive >= CONFIRM_FRAMES
        hud_color = (0, 255, 0) if confirmed else (0, 165, 255)
        cv2.putText(frame,
                    f"Arrow: {direction}  ({consecutive}/{CONFIRM_FRAMES})",
                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, hud_color, 2)
        cv2.putText(frame, "ESC = abort",
                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.imshow("cv.py", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            raise SystemExit("Aborted during scan.")

        if confirmed:
            print(f"Arrow confirmed: {last_dir}")
            return last_dir

# ─────────────────────────────────────────────────────────────────────────────
# Between phases 2 and 3 – wait for operator to confirm drive start
# ─────────────────────────────────────────────────────────────────────────────
def wait_for_drive_start(cap, direction: str) -> None:
    """Show a ready screen after the turn; wait for SPACE to begin driving."""
    print("=== Turn complete.  Press SPACE in the window to begin driving. ===")
    while True:
        ret, frame = cap.read()
        if not ret:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)

        cv2.putText(frame, f"Turn done — facing {direction} ball",
                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, f"Heading: {math.degrees(POSE_TH):.1f} deg",
                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, "SPACE = start driving   ESC = abort",
                    (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)
        cv2.imshow("cv.py", frame)

        key = cv2.waitKey(30) & 0xFF
        if key == ord(' '):
            return
        if key == 27:
            raise SystemExit("Aborted before driving.")

# ─────────────────────────────────────────────────────────────────────────────
# Phase 3 – Drive forward with heading-hold until SPACE
# ─────────────────────────────────────────────────────────────────────────────
DRIVE_SPEED   = 6.0    # rad/s wheel speed
K_THETA       = 8.0    # proportional gain on heading error
K_OMEGA       = 0.5    # damping on yaw rate
MAX_WHEEL_CMD = 10.0


def drive_until_stop(cap) -> None:
    """
    Drive forward with heading-hold.  Show camera feed.
    Press SPACE or ESC to stop.
    """
    print("=== PHASE 3: Driving.  Press SPACE to stop. ===")
    target_th = POSE_TH
    last_t    = time.monotonic()

    while True:
        now    = time.monotonic()
        dt     = now - last_t
        last_t = now

        update_pose(dt)

        # Heading-hold correction
        err_th = _wrap_pi(POSE_TH - target_th)
        wl, wr = read_wheel_vel()
        omega  = WHEEL_RADIUS_IN * (wl - wr) / WHEEL_BASE_IN
        corr   = K_THETA * err_th + K_OMEGA * omega

        cmd_l = max(0.0, min(MAX_WHEEL_CMD, DRIVE_SPEED - corr))
        cmd_r = max(0.0, min(MAX_WHEEL_CMD, DRIVE_SPEED + corr))
        set_wheel_velocity(cmd_l, cmd_r)

        print(
            f"[DRIVE] th={math.degrees(POSE_TH):+.1f} deg  "
            f"err={math.degrees(err_th):+.2f} deg  "
            f"cmdL={cmd_l:.2f}  cmdR={cmd_r:.2f}"
        )

        # Camera feed (non-blocking waitKey)
        ret, frame = cap.read()
        if ret:
            cv2.putText(frame,
                        f"DRIVING  th={math.degrees(POSE_TH):+.1f} deg  "
                        f"err={math.degrees(err_th):+.1f} deg",
                        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
            cv2.putText(frame, "SPACE / ESC = stop",
                        (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)
            cv2.imshow("cv.py", frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord(' '), 27):
            break

        time.sleep(0.002)

    stop()
    print("=== Stopped. ===")

# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main() -> None:
    global POSE_X, POSE_Y, POSE_TH

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera (device 0).")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    try:
        # ── Phase 1: detect arrow ────────────────────────────────────────────
        direction = scan_for_arrow(cap)   # "LEFT" or "RIGHT"

        # ── Phase 2: turn 90° toward the ball ────────────────────────────────
        # Arrow LEFT  → ball is to our left  → turn CCW (+90°)
        # Arrow RIGHT → ball is to our right → turn CW  (-90°)
        turn_deg = 90.0 if direction == "LEFT" else -90.0
        print(f"\n=== PHASE 2: Turning {turn_deg:+.0f} deg toward {direction} ball ===")
        POSE_X = POSE_Y = POSE_TH = 0.0
        turn_angle(turn_deg)

        # ── Wait for operator confirmation ───────────────────────────────────
        wait_for_drive_start(cap, direction)

        # ── Phase 3: drive toward the ball ───────────────────────────────────
        drive_until_stop(cap)

    finally:
        stop()
        cap.release()
        cv2.destroyAllWindows()
        print("cv.py finished.")


if __name__ == "__main__":
    main()
