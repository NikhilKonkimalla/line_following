# NewODO.py
# Executes commands from path_planning.py:
#   ("TURN", degrees)   + = CCW, - = CW
#   ("DRIVE", inches)
#
# This version:
# - Maintains and prints odometry pose (x,y,theta) while moving
# - Uses measured dt (monotonic clock) so encoder integration is weighted correctly
# - TRUE heading-hold during DRIVE based on theta error (NOT wr-wl)
# - Turn uses PID on remaining angle + optional brake pulse (NEW)
#
# If you still see drift, it is likely wheel slip / wheel radius mismatch (odometry can't "fix" that),
# but this heading-hold will actively correct the robot's yaw based on odometry theta.

import time
import math
import board
from motorgo import Plink, ControlMode

print("LOADED NewODO.py from:", __file__)

# ------------------------------
# Hardware init
# ------------------------------
i2c = board.I2C()

plink = Plink()
plink.power_supply_voltage = 9.6

# Make sure these match your wiring
left_motor = plink.channel1
right_motor = plink.channel3

left_motor.motor_voltage_limit = 6.0
right_motor.motor_voltage_limit = 6.0

plink.connect()

left_motor.control_mode = ControlMode.VELOCITY
right_motor.control_mode = ControlMode.VELOCITY

# You may need to tune these for your motors
left_motor.set_velocity_pid_gains(4.5, 0, 0.0)
right_motor.set_velocity_pid_gains(4.5, 0, 0.0)

# ------------------------------
# MOTOR DIRECTION (CHANGE HERE)
# ------------------------------
# If DRIVE forward goes backward, flip BOTH.
# If TURN behaves like DRIVE, one sign is wrong.
LEFT_MOTOR_SIGN = 1
RIGHT_MOTOR_SIGN = -1

# Optional straight-line scaling (tiny calibration)
LEFT_WHEEL_SCALE = 1.00
RIGHT_WHEEL_SCALE = 1.00

# ------------------------------
# Robot params (inches)
# ------------------------------
WHEEL_RADIUS_IN = 1.15
WHEEL_BASE_IN = 5.38

# If wheel speed = motor speed * (24/40)
GEAR_WHEEL_PER_MOTOR = 24 / 40
GEAR_MOTOR_PER_WHEEL = 40 / 24

# ------------------------------
# Global pose (odometry)
# ------------------------------
POSE_X = 0.0     # inches
POSE_Y = 0.0     # inches
POSE_TH = 0.0    # radians


def reset_pose(x=0.0, y=0.0, theta_rad=0.0):
    global POSE_X, POSE_Y, POSE_TH
    POSE_X = float(x)
    POSE_Y = float(y)
    POSE_TH = float(theta_rad)


def pose_str() -> str:
    return f"x={POSE_X:.2f} in, y={POSE_Y:.2f} in, th={math.degrees(POSE_TH):.1f}°"


def _set_motor_velocity(motor, w_motor_rad_s: float):
    # Support both motorgo API styles
    if hasattr(motor, "velocity_command"):
        motor.velocity_command = w_motor_rad_s
    else:
        motor.set_velocity(w_motor_rad_s)


def set_wheel_velocity(wl_wheel_rad_s: float, wr_wheel_rad_s: float):
    """
    Command wheel angular velocities (rad/s) in the WHEEL frame.
    Scales apply first, then sign mapping to motor frame.
    """
    wl_wheel_rad_s *= LEFT_WHEEL_SCALE
    wr_wheel_rad_s *= RIGHT_WHEEL_SCALE

    motor_left = LEFT_MOTOR_SIGN * wl_wheel_rad_s * GEAR_MOTOR_PER_WHEEL
    motor_right = RIGHT_MOTOR_SIGN * wr_wheel_rad_s * GEAR_MOTOR_PER_WHEEL

    _set_motor_velocity(left_motor, motor_left)
    _set_motor_velocity(right_motor, motor_right)


def stop():
    _set_motor_velocity(left_motor, 0.0)
    _set_motor_velocity(right_motor, 0.0)


def read_wheel_vel():
    """
    Read wheel angular velocities (rad/s) in the SAME convention used by set_wheel_velocity().
    """
    wl = LEFT_MOTOR_SIGN * (left_motor.velocity * GEAR_WHEEL_PER_MOTOR)
    wr = RIGHT_MOTOR_SIGN * (right_motor.velocity * GEAR_WHEEL_PER_MOTOR)
    return wl, wr


def _wrap_pi(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _slew_toward(current: float, target: float, max_delta: float) -> float:
    return current + _clamp(target - current, -max_delta, max_delta)


def update_pose(dt: float):
    """
    Integrate pose using encoder wheel velocities (dead-reckoning).

    - Reads wheel angular velocities from the motor controllers (encoders).
    - Converts them to linear (v) and angular (omega) body velocities.
    - Integrates forward for dt seconds.
    """
    global POSE_X, POSE_Y, POSE_TH

    wl, wr = read_wheel_vel()
    v = WHEEL_RADIUS_IN * (wl + wr) / 2.0                # in/s
    omega = WHEEL_RADIUS_IN * (wr - wl) / WHEEL_BASE_IN  # rad/s

    POSE_TH = _wrap_pi(POSE_TH + omega * dt)
    POSE_X += v * math.cos(POSE_TH) * dt
    POSE_Y += v * math.sin(POSE_TH) * dt


def drive_distance(
    dist_in: float,
    wheel_speed_rad_s: float = 6.0,
    heading_hold: bool = True,
    line_hold: bool = True,
    k_theta: float = 8.0,
    k_omega: float = 0.5,
    k_cte: float = 0.8,
    timeout_s: float = 25.0,
    print_hz: float = 20.0,
    soft_start_s: float = 0.25,
    max_wheel_accel: float = 8.0,
):
    """
    Drive forward dist_in inches with optional heading + line hold.
    Uses startup ramp + slew-limited wheel commands to reduce slip on segment entry.
    """
    if dist_in == 0:
        return

    start_x = POSE_X
    start_y = POSE_Y
    start_th = POSE_TH

    # unit vector along the desired line (initial heading)
    ux = math.cos(start_th)
    uy = math.sin(start_th)

    t0 = time.monotonic()
    last_t = t0

    next_print = t0
    print_period = 1.0 / max(1e-6, print_hz)

    dist_in = float(dist_in)
    direction = 1.0 if dist_in >= 0 else -1.0
    dist_in = abs(dist_in)
    cmd_l_sent = 0.0
    cmd_r_sent = 0.0

    while True:
        now = time.monotonic()
        dt = now - last_t
        last_t = now

        update_pose(dt)

        dx = POSE_X - start_x
        dy = POSE_Y - start_y
        traveled = math.hypot(dx, dy)

        # Heading error to initial heading
        err_th = _wrap_pi(start_th - POSE_TH)

        # Yaw rate estimate from wheel speeds (odometry-based)
        wl, wr = read_wheel_vel()
        omega = WHEEL_RADIUS_IN * (wr - wl) / WHEEL_BASE_IN  # rad/s

        # Cross-track error to the initial line through start, along (ux,uy)
        # signed perp distance: (p-start) x u
        cte = (dx * uy - dy * ux) if line_hold else 0.0

        # Compose correction
        corr = 0.0
        hold_str = "none"
        if heading_hold and line_hold:
            hold_str = "theta+line"
            corr = k_theta * err_th + (-k_omega * omega) + (k_cte * cte)
        elif heading_hold:
            hold_str = "theta"
            corr = k_theta * err_th + (-k_omega * omega)
        elif line_hold:
            hold_str = "line"
            corr = (k_cte * cte) + (-k_omega * omega)

        ramp = 1.0 if soft_start_s <= 0 else min(1.0, (now - t0) / soft_start_s)
        base = direction * wheel_speed_rad_s
        cmd_l_target = (base - corr) * ramp
        cmd_r_target = (base + corr) * ramp

        if max_wheel_accel <= 0:
            cmd_l_sent = cmd_l_target
            cmd_r_sent = cmd_r_target
        else:
            max_delta = max_wheel_accel * max(0.0, dt)
            cmd_l_sent = _slew_toward(cmd_l_sent, cmd_l_target, max_delta)
            cmd_r_sent = _slew_toward(cmd_r_sent, cmd_r_target, max_delta)

        set_wheel_velocity(cmd_l_sent, cmd_r_sent)

        if now >= next_print:
            print(
                f"[DRIVE] {pose_str()} traveled={traveled:.2f}/{dist_in:.2f}  hold={hold_str}  "
                f"err_th={math.degrees(err_th):+.2f}° cte={cte:+.2f} in  "
                f"omega={math.degrees(omega):+.1f}°/s corr={corr:+.2f}  "
                f"ramp={ramp:.2f} cmdL={cmd_l_sent:.2f} cmdR={cmd_r_sent:.2f}"
            )
            next_print += print_period

        if traveled >= dist_in:
            break

        if now - t0 > timeout_s:
            stop()
            raise RuntimeError("Drive timed out (not moving as expected).")

        time.sleep(0.002)

    stop()
    time.sleep(0.05)


def turn_angle(
    deg: float,
    max_turn_speed: float = 6.0,
    min_turn_speed: float = 1.5,
    slow_down_deg: float = 20.0,   # kept for compatibility; PID already naturally slows
    timeout_s: float = 12.0,
    print_hz: float = 10.0,
    use_brake_pulse: bool = False,
    brake_speed: float = 2.0,
    brake_time: float = 0.01,
    # --- PID gains ---
    kp: float = 3,     # (speed per rad of remaining angle)
    ki: float = 0,
    kd: float = 0,    # (speed per rad/s of remaining angle change)
    i_limit: float = 2.0,     # clamp integral
    stop_tol_deg: float = 0,  # stop tolerance in degrees
    soft_start_s: float = 0.25,
    max_wheel_accel: float = 4.0,
):
    """
    Turn in place by deg degrees (+CCW, -CW) using encoder odometry.
    PID is on remaining angle magnitude, output is wheel speed (rad/s).

    IMPORTANT: DOES NOT change channels or motor sign conventions.
    The actual motor command line remains:
        set_wheel_velocity(-sign * speed, +sign * speed)
    """
    if deg == 0:
        return
    if deg == 180:
        deg = 179

    target = math.radians(deg)
    start_th = POSE_TH

    t0 = time.monotonic()
    last_t = t0

    next_print = t0
    print_period = 1.0 / max(1e-6, print_hz)

    # KEEP your exact convention
    sign = 1.0 if deg < 0 else -1.0

    integ = 0.0
    prev_e = None
    stop_tol = math.radians(abs(stop_tol_deg))
    cmd_l_sent = 0.0
    cmd_r_sent = 0.0

    while True:
        now = time.monotonic()
        dt = now - last_t
        last_t = now

        update_pose(dt)

        # Signed turned angle relative to start (wrapped into [-pi,pi]).
        turned = _wrap_pi(POSE_TH - start_th)

        # Match your original "abs(target)-abs(turned)" style (works for <=180°)
        remaining = max(0.0, abs(target) - abs(turned))

        if remaining <= stop_tol:
            break

        # PID error is remaining angle (radians), always positive
        e = remaining

        if prev_e is None:
            de = 0.0
        else:
            de = (e - prev_e) / max(1e-6, dt)
        prev_e = e

        integ += e * dt
        if integ > i_limit:
            integ = i_limit

        # PID output -> speed command (wheel rad/s)
        speed = kp * e + ki * integ + kd * de

        # clamp & stiction floor
        if speed > max_turn_speed:
            speed = max_turn_speed
        if speed < min_turn_speed:
            speed = min_turn_speed

        ramp = 1.0 if soft_start_s <= 0 else min(1.0, (now - t0) / soft_start_s)
        speed_cmd = speed * ramp

        # Keep existing turn sign convention; only soften command onset.
        cmd_l_target = -sign * speed_cmd
        cmd_r_target = +sign * speed_cmd

        if max_wheel_accel <= 0:
            cmd_l_sent = cmd_l_target
            cmd_r_sent = cmd_r_target
        else:
            max_delta = max_wheel_accel * max(0.0, dt)
            cmd_l_sent = _slew_toward(cmd_l_sent, cmd_l_target, max_delta)
            cmd_r_sent = _slew_toward(cmd_r_sent, cmd_r_target, max_delta)

        set_wheel_velocity(cmd_l_sent, cmd_r_sent)

        if now >= next_print:
            wl, wr = read_wheel_vel()
            print(
                f"[TURN ] {pose_str()} turned={math.degrees(turned):.1f}/{deg:.1f}  "
                f"rem={math.degrees(remaining):.2f}°  wl={wl:+.2f} wr={wr:+.2f}  "
                f"ramp={ramp:.2f} speed={speed_cmd:.2f}"
            )
            next_print += print_period

        if now - t0 > timeout_s:
            stop()
            raise RuntimeError("Turn timed out (likely not rotating—check motor signs).")

        time.sleep(0.002)

    if use_brake_pulse:
        set_wheel_velocity(+sign * brake_speed, -sign * brake_speed)
        time.sleep(brake_time)

    stop()
    time.sleep(0.05)


def execute_commands(
    cmds,
    drive_speed: float = 6.0,
    turn_max_speed: float = 5.0,
    heading_hold: bool = True,
    line_hold: bool = True,
    k_theta: float = 8.0,
    k_omega: float = 0.5,
    k_cte: float = 0.8,
    drive_soft_start_s: float = 0.25,
    turn_soft_start_s: float = 0.25,
    drive_max_wheel_accel: float = 8.0,
    turn_max_wheel_accel: float = 4.0,
):
    """
    Execute list of ("TURN", deg) and ("DRIVE", inches).
    Heading hold is ON by default for DRIVE segments.
    """
    for typ, val in cmds:
        if typ == "TURN":
            print(f"\n=== TURN {val:+.0f}° ===")
            turn_angle(
                val,
                max_turn_speed=turn_max_speed,
                soft_start_s=turn_soft_start_s,
                max_wheel_accel=turn_max_wheel_accel,
            )
        elif typ == "DRIVE":
            print(f"\n=== DRIVE {val:.2f} in ===  (heading_hold={heading_hold}, line_hold={line_hold}, k_theta={k_theta}, k_omega={k_omega}, k_cte={k_cte})")
            drive_distance(
                val,
                wheel_speed_rad_s=drive_speed,
                heading_hold=heading_hold,
                line_hold=line_hold,
                k_theta=k_theta,
                k_omega=k_omega,
                k_cte=k_cte,
                soft_start_s=drive_soft_start_s,
                max_wheel_accel=drive_max_wheel_accel,
            )
        else:
            raise ValueError(f"Unknown command type: {typ}")

    print("\nDONE. Final pose:", pose_str())


def quick_straight_test(dist_in=60.0):
    """
    Run this to tune heading-hold:
      - If drift still accumulates: increase k_theta (8 -> 10 -> 12)
      - If it oscillates/snakes: reduce k_theta or increase k_omega slightly
    """
    reset_pose(0.0, 0.0, 0.0)
    execute_commands([("DRIVE", float(dist_in))], drive_speed=6.0, heading_hold=True, k_theta=10.0, k_omega=0.6)


if __name__ == "__main__":
    reset_pose(0.0, 0.0, 0.0)
    demo = [('TURN', -90.0), ('DRIVE', 6.0), ('TURN', -90.0), ('DRIVE', 8.0), ('TURN', 90.0), ('DRIVE', 20.0), ('TURN', 90.0), ('DRIVE', 6.0), ('TURN', -90.0), ('DRIVE', 33.0), ('TURN', -90.0), ('DRIVE', 10.0)]
    execute_commands(
        demo,
        drive_speed=6.0,
        heading_hold=True,
        k_theta=12.0,
        k_omega=0.6,
    )