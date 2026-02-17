# NewODO.py
# Executes commands from path_planning.py:
#   ("TURN", degrees)   + = CCW, - = CW
#   ("DRIVE", inches)
#
# This version:
# - Maintains and prints odometry pose (x,y,theta) while moving
# - Uses measured dt (monotonic clock) so encoder integration is weighted correctly
# - TRUE heading-hold during DRIVE based on theta error (NOT wr-wl)
# - Turn uses a slowdown ramp + optional brake pulse to reduce overshoot
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
WHEEL_BASE_IN = 5.25

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


def update_pose(dt: float):
    """
    Integrate pose using encoder wheel velocities (dead-reckoning).

    - Reads wheel angular velocities from the motor controllers (encoders).
    - Converts them to linear (v) and angular (omega) body velocities.
    - Integrates forward for dt seconds.

    Note: this is *pure odometry* (no external sensors). If the wheels slip,
    the pose estimate will drift even if the controller is doing the right thing.
    """
    global POSE_X, POSE_Y, POSE_TH

    wl, wr = read_wheel_vel()
    v = WHEEL_RADIUS_IN * (wl + wr) / 2.0                # in/s
    omega = WHEEL_RADIUS_IN * (wl - wr) / WHEEL_BASE_IN  # rad/s

    POSE_X += v * math.cos(POSE_TH) * dt
    POSE_Y += v * math.sin(POSE_TH) * dt
    POSE_TH = _wrap_pi(POSE_TH + omega * dt)


def drive_distance(
    dist_in: float,
    wheel_speed_rad_s: float = 6.0,
    timeout_s: float = 25.0,
    print_hz: float = 10.0,
    heading_hold: bool = True,
    line_hold: bool = True,
    k_theta: float = 8.0,         # P gain on heading error (rad -> wheel rad/s)
    k_omega: float = 0.5,         # D-ish damping on yaw rate (rad/s -> wheel rad/s)
    k_cte: float = 0.8,           # P gain on cross-track error (in -> wheel rad/s)
    max_wheel_cmd: float = 10.0,  # clamp wheel command
):
    """
    Drive straight for dist_in inches using encoder-integrated odometry.

    What this loop tries to hold during a DRIVE segment:
      1) Heading hold (theta): keep POSE_TH near its value at the start of the segment.
      2) Line hold (cross-track): keep the robot on the line it started on.

    Line hold is what you're asking for:
      - If the robot is driving along +X/-X (east/west), it will keep Y ≈ start_y.
      - If the robot is driving along +Y/-Y (north/south), it will keep X ≈ start_x.
    More generally, it keeps you on the infinite line that starts at (start_x,start_y)
    and points along target_th.

    Tuning tips:
      - Still drifts sideways off the line? Increase k_cte a bit (0.8 -> 1.0 -> 1.2)
      - Snakes/oscillates? Decrease k_theta and/or k_cte, or increase k_omega slightly.
    """
    if dist_in <= 0:
        return

    # --- Segment reference (the "ideal" line) ---
    start_x, start_y = POSE_X, POSE_Y
    target_th = POSE_TH  # heading we want to maintain during this segment

    # Unit direction of the desired line (along-track) and its left normal (cross-track).
    #   u  = [cos(th), sin(th)]
    #   nL = [-sin(th), cos(th)]  (points to the left of u)
    ux = math.cos(target_th)
    uy = math.sin(target_th)
    nxL = uy
    nyL = -ux

    t0 = time.monotonic()
    last_t = t0

    next_print = t0
    print_period = 1.0 / max(1e-6, print_hz)

    # --- Initial command ---
    set_wheel_velocity(wheel_speed_rad_s, wheel_speed_rad_s)

    # --- Debug values for printing ---
    err_th = 0.0
    omega = 0.0
    cte = 0.0
    corr = 0.0
    cmd_l = wheel_speed_rad_s
    cmd_r = wheel_speed_rad_s

    while True:
        now = time.monotonic()
        dt = now - last_t
        last_t = now

        # 1) Integrate pose from encoders (dead-reckoning)
        update_pose(dt)

        # 2) Track progress along the segment
        dx = POSE_X - start_x
        dy = POSE_Y - start_y
        traveled = math.hypot(dx, dy)

        # 3) Optional closed-loop correction (heading + line)
        if heading_hold or line_hold:
            # Heading error relative to start of the segment
            err_th = _wrap_pi(POSE_TH - target_th )

            # Yaw rate estimate from wheel encoders (rad/s)
            wl, wr = read_wheel_vel()
            omega = WHEEL_RADIUS_IN * (wl - wr) / WHEEL_BASE_IN

            # Cross-track error (inches):
            #   positive => robot is LEFT of the desired line
            #   negative => robot is RIGHT of the desired line
            #
            # cte = (p - p0) dot nL
            cte = dx * nxL + dy * nyL

            # Differential steering correction:
            #   positive corr => CCW (left) turn (right wheel faster)
            # For cross-track:
            #   if cte > 0 (left of line), we want a RIGHT turn => corr should go negative.
            corr = 0.0
            if heading_hold:
                corr += (k_theta * err_th - k_omega * omega)
            if line_hold:
                corr += (-k_cte * cte)

            cmd_l = wheel_speed_rad_s - corr
            cmd_r = wheel_speed_rad_s + corr

            # Clamp so we never command negative wheel speeds in this simple forward-only driver
            cmd_l = max(0.0, min(max_wheel_cmd, cmd_l))
            cmd_r = max(0.0, min(max_wheel_cmd, cmd_r))

            set_wheel_velocity(cmd_l, cmd_r)
        else:
            # Open-loop: just drive both wheels the same speed
            set_wheel_velocity(wheel_speed_rad_s, wheel_speed_rad_s)

        # 4) Print status
        if now >= next_print:
            flags = []
            if heading_hold:
                flags.append("theta")
            if line_hold:
                flags.append("line")
            hold_str = "+".join(flags) if flags else "none"

            print(
                f"[DRIVE] {pose_str()} traveled={traveled:.2f}/{dist_in:.2f}  hold={hold_str}  "
                f"err_th={math.degrees(err_th):+.2f}° cte={cte:+.2f} in  "
                f"omega={math.degrees(omega):+.1f}°/s corr={corr:+.2f}  "
                f"cmdL={cmd_l:.2f} cmdR={cmd_r:.2f}"
            )
            next_print += print_period

        # 5) Stop conditions
        if traveled >= dist_in:
            break

        if now - t0 > timeout_s:
            stop()
            raise RuntimeError("Drive timed out (not moving as expected).")

        # Small sleep to avoid pegging CPU; still fast enough for control
        time.sleep(0.002)

    stop()
    time.sleep(0.05)


def turn_angle(
    deg: float,
    max_turn_speed: float = 5.0,
    min_turn_speed: float = 1.5,
    slow_down_deg: float = 30.0,
    timeout_s: float = 12.0,
    print_hz: float = 10.0,
    use_brake_pulse: bool = True,
    brake_speed: float = 2.0,
    brake_time: float = 0.01,
):
    """
    Turn in place by deg degrees (+CCW, -CW) using encoder odometry.
    Slows down near target to reduce overshoot + optional brake pulse.
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

    sign = 1.0 if deg < 0 else -1.0

    while True:
        now = time.monotonic()
        dt = now - last_t
        last_t = now

        update_pose(dt)

        # Signed turned angle relative to start (wrapped into [-pi,pi]).
        # Works well for turns up to ~180° in one command.
        turned = _wrap_pi(POSE_TH - start_th)

        remaining = abs(target) - abs(turned)
        remaining_deg = math.degrees(max(0.0, remaining))

        if remaining_deg < slow_down_deg:
            frac = remaining_deg / slow_down_deg
            speed = min_turn_speed + frac * (max_turn_speed - min_turn_speed)
        else:
            speed = max_turn_speed

        # CCW (+): left backward, right forward
        set_wheel_velocity(-sign * speed, +sign * speed)

        if now >= next_print:
            wl, wr = read_wheel_vel()
            print(
                f"[TURN ] {pose_str()} turned={math.degrees(turned):.1f}/{deg:.1f}  "
                f"wl={wl:+.2f} wr={wr:+.2f} speed={speed:.2f}"
            )
            next_print += print_period

        if abs(turned) >= abs(target):
            break

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
):
    """
    Execute list of ("TURN", deg) and ("DRIVE", inches).
    Heading hold is ON by default for DRIVE segments.
    """
    for typ, val in cmds:
        if typ == "TURN":
            print(f"\n=== TURN {val:+.0f}° ===")
            turn_angle(val, max_turn_speed=turn_max_speed)
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
    # Example test: straight drive with heading hold
    reset_pose(0.0, 0.0, 0.0)
    demo = [('TURN', 180.0), ('DRIVE', 4.0), ('TURN', 90.0), ('DRIVE', 9.0), ('TURN', -90.0), ('DRIVE', 22.0), ('TURN', -90.0), ('DRIVE', 8.0), ('TURN', 90.0), ('DRIVE', 32.0), ('TURN', 90.0), ('DRIVE', 7.0)]    
    execute_commands(
        demo,
        drive_speed=6.0,
        heading_hold=True,
        k_theta=12.0,  # start here; try 12 if it still drifts
        k_omega=0.6,
    )

    # Example full command list:
    # cmds = [("DRIVE", 24.0), ("TURN", 90.0), ("DRIVE", 12.0)]
    # execute_commands(cmds)
