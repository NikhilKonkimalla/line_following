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
left_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)
right_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)

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
WHEEL_BASE_IN = 6.0

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
    Integrate pose using encoder wheel velocities.
    dt is measured seconds.
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
    k_theta: float = 8.0,        # P gain on heading error (rad -> wheel rad/s)
    k_omega: float = 0.5,        # D-ish damping on yaw rate (rad/s -> wheel rad/s)
    max_wheel_cmd: float = 10.0, # clamp wheel command
):
    """
    Drive straight for dist_in inches using encoder-integrated odometry.
    TRUE heading-hold: keeps theta near its value at drive start using theta error.

    Tuning:
      - If it still drifts: increase k_theta (8 -> 10 -> 12)
      - If it "snakes" / oscillates: reduce k_theta or increase k_omega
    """
    if dist_in <= 0:
        return

    start_x, start_y = POSE_X, POSE_Y
    target_th = POSE_TH  # heading we want to maintain during this straight segment

    t0 = time.monotonic()
    last_t = t0

    next_print = t0
    print_period = 1.0 / max(1e-6, print_hz)

    # initial command
    set_wheel_velocity(wheel_speed_rad_s, wheel_speed_rad_s)

    # for printing
    err_th = 0.0
    omega = 0.0
    corr = 0.0
    cmd_l = wheel_speed_rad_s
    cmd_r = wheel_speed_rad_s

    while True:
        now = time.monotonic()
        dt = now - last_t
        last_t = now

        # integrate pose from encoders
        update_pose(dt)

        dx = POSE_X - start_x
        dy = POSE_Y - start_y
        traveled = math.hypot(dx, dy)

        if heading_hold:
            # heading error relative to the heading at the start of the segment
            err_th = _wrap_pi(POSE_TH - target_th)

            # yaw rate estimate from wheel encoders
            wl, wr = read_wheel_vel()
            omega = WHEEL_RADIUS_IN * (wl - wr) / WHEEL_BASE_IN  # rad/s

            # steering correction (differential wheel speed)
            corr = k_theta * err_th + k_omega * omega

            cmd_l = wheel_speed_rad_s - corr
            cmd_r = wheel_speed_rad_s + corr

            cmd_l = max(0.0, min(max_wheel_cmd, cmd_l))
            cmd_r = max(0.0, min(max_wheel_cmd, cmd_r))

            set_wheel_velocity(cmd_l, cmd_r)
        else:
            set_wheel_velocity(wheel_speed_rad_s, wheel_speed_rad_s)

        if now >= next_print:
            if heading_hold:
                print(
                    f"[DRIVE] {pose_str()} traveled={traveled:.2f}/{dist_in:.2f}  "
                    f"err_th={math.degrees(err_th):+.2f}° omega={math.degrees(omega):+.1f}°/s  "
                    f"corr={corr:+.2f} cmdL={cmd_l:.2f} cmdR={cmd_r:.2f}"
                )
            else:
                print(f"[DRIVE] {pose_str()} traveled={traveled:.2f}/{dist_in:.2f} (heading_hold OFF)")
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
    max_turn_speed: float = 5.0,
    min_turn_speed: float = 1.5,
    slow_down_deg: float = 25.0,
    timeout_s: float = 12.0,
    print_hz: float = 10.0,
    use_brake_pulse: bool = True,
    brake_speed: float = 2.0,
    brake_time: float = 0.08,
):
    """
    Turn in place by deg degrees (+CCW, -CW) using encoder odometry.
    Slows down near target to reduce overshoot + optional brake pulse.
    """
    if deg == 0:
        return

    target = math.radians(deg)
    start_th = POSE_TH

    t0 = time.monotonic()
    last_t = t0

    next_print = t0
    print_period = 1.0 / max(1e-6, print_hz)

    sign = 1.0 if deg > 0 else -1.0

    while True:
        now = time.monotonic()
        dt = now - last_t
        last_t = now

        update_pose(dt)

        # signed turned angle relative to start (wrapped)
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
    k_theta: float = 8.0,
    k_omega: float = 0.5,
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
            print(f"\n=== DRIVE {val:.2f} in ===  (heading_hold={heading_hold}, k_theta={k_theta}, k_omega={k_omega})")
            drive_distance(
                val,
                wheel_speed_rad_s=drive_speed,
                heading_hold=heading_hold,
                k_theta=k_theta,
                k_omega=k_omega,
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
    demo = [('DRIVE', 55.0), ('TURN', -90.0), ('DRIVE', 35.0)]
    execute_commands(
        demo,
        drive_speed=6.0,
        heading_hold=True,
        k_theta=10.0,  # start here; try 12 if it still drifts
        k_omega=0.6,
    )

    # Example full command list:
    # cmds = [("DRIVE", 24.0), ("TURN", 90.0), ("DRIVE", 12.0)]
    # execute_commands(cmds)
