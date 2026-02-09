import time
import math
import board
from motorgo import Plink, ControlMode

i2c = board.I2C()

plink = Plink()
plink.power_supply_voltage = 9.6

left_motor = plink.channel1
right_motor = plink.channel3

left_motor.motor_voltage_limit = 6.0
right_motor.motor_voltage_limit = 6.0

plink.connect()
imu = plink.imu

left_motor.control_mode = ControlMode.POWER
right_motor.control_mode = ControlMode.POWER

# Calibrate gyro bias
print("Keep robot still: calibrating gyro...")
time.sleep(1.0)

N = 300
gx_sum = 0.0
dt_cal = 0.005
for _ in range(N):
    g = imu.gyro
    gx_sum += g[1] 
    time.sleep(dt_cal)

gyro_bias_coarse = gx_sum / N
print("Gyro bias (coarse):", gyro_bias_coarse)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

#Should be calculating pitch about the x-axis
def accel_pitch(ax, ay, az):
    return math.degrees(math.atan2(ax, az))


# -----------------------
# Kalman filter (angle + gyro bias)
# -----------------------
# State:
#   x[0] = angle (deg)
#   x[1] = gyro bias (deg/s)  (this is the *residual* bias after coarse calibration)
#
# Tuning params:
#   Q_angle: how much you expect the true angle to wander vs your model (process noise)
#   Q_bias : how much you expect gyro bias to drift over time
#   R_meas : how noisy the accelerometer angle measurement is
Q_angle = 0.001
Q_bias  = 0.003
R_meas  = 0.5

theta = 0.0     # filter output angle (deg)
bias  = 0.0     # residual bias estimate (deg/s)

# Covariance matrix P (2x2)
P00, P01 = 1.0, 0.0
P10, P11 = 0.0, 1.0

def kalman_update(theta_meas_deg, gyro_rate_deg_s, dt):
    """
    One update of the 2-state (angle + bias) Kalman filter.
    Inputs:
      theta_meas_deg: angle from accel (deg)
      gyro_rate_deg_s: gyro rate (deg/s) AFTER coarse bias removal
      dt: timestep (s)
    Returns:
      theta_est (deg), bias_est (deg/s)
    """
    global theta, bias, P00, P01, P10, P11

    # ---------- Predict ----------
    # Use gyro minus estimated residual bias to predict angle forward.
    rate = gyro_rate_deg_s - bias
    theta = theta + dt * rate

    # Update covariance P = A P A^T + Q
    # A = [[1, -dt],
    #      [0,  1 ]]
    P00 = P00 + dt * (dt*P11 - P01 - P10) + Q_angle
    P01 = P01 - dt * P11
    P10 = P10 - dt * P11
    P11 = P11 + Q_bias

    # ---------- Update (Correct) ----------
    # Innovation
    y = theta_meas_deg - theta

    # Innovation covariance
    S = P00 + R_meas

    # Kalman gain
    K0 = P00 / S
    K1 = P10 / S

    # State update
    theta = theta + K0 * y
    bias  = bias  + K1 * y

    # Covariance update (Joseph form not necessary here; standard works fine)
    P00_old = P00
    P01_old = P01

    P00 = P00 - K0 * P00_old
    P01 = P01 - K0 * P01_old
    P10 = P10 - K1 * P00_old
    P11 = P11 - K1 * P01_old

    return theta, bias


# PD/PID controller
theta_set = 0.0  # adjust for any base theta error

Kp = 1
Kd = 2.5
Ki = 0
deadzone = 5 #in degrees

integral = 0.0
integral_limit = 0.0 #0 for now, we will determine if we need it later

max_u = 6.0  # voltage limit
pushP = 1
pushD = 1
pipeD = []
pipeP = []

# Main loop timing
dt_target = 0.01  # 200 Hz
t_prev = time.monotonic()

try:
    while True:
        t_now = time.monotonic()
        dt = t_now - t_prev
        if dt <= 0:
            continue
        t_prev = t_now

        # Read our IMU
        gx, gy, gz = imu.gyro
        ax, ay, az = imu.accel

        # Gyro rate (deg/s): remove obvious bias, Kalman will learn residual bias
        gyro_rate = gx - gyro_bias_coarse  # <--adjust if we change Pi orientation

        # Accel-based angle measurement (deg)
        theta_acc = accel_pitch(ax, ay, az)

        # Apply Kalman fuse
        theta_est, bias_est = kalman_update(theta_acc, gyro_rate, dt)

        # Control error (i.e. how much the robot has tipped)
        error = (theta_est - theta_set)

        #Here just in case we want to use it
        integral += error * dt
        integral = clamp(integral, -integral_limit, integral_limit)

        #This does a sort of sliding window that validates all three
        #Readings to make sure they are the same sign so that the freaky
        #Shaking doesn't occur.
        # if len(pipeP) < 3:
        #     pipeP.append(theta)
        # elif (0 <= x for x in pipeP) or (x <= 0 for x in pipeP):
        #     pipeP.pop(0)
        #     pipeP.append(theta)
        #     pushP = 1
        # else:
        #     pipeP.pop(0)
        #     pipeP.append(theta)
        #     pushP = 0.1
        
        # if len(pipeD) < 3:
        #     pipeD.append(gx)
        # elif (0 <= x for x in pipeD) or (x <= 0 for x in pipeD):
        #     pipeD.pop(0)
        #     pipeD.append(gx)
        #     pushD = 1
        # else:
        #     pipeD.pop(0)
        #     pipeD.append(gx)
        #     pushD = 0.1

        
        u = pushP * Kp * error + pushD * Kd * gyro_rate + Ki * integral
        u = clamp(u, -max_u, max_u)

        left_motor.power_command = -u
        right_motor.power_command = u

        # This stops the robot if it has already fallen over
        if abs(theta_est) > 80:
            left_motor.power_command = 0.0
            right_motor.power_command = 0.0
            integral = 0.0

        # Maintain loop rate
        sleep_time = dt_target - (time.monotonic() - t_now)
        if sleep_time > 0:
            time.sleep(sleep_time)

        print(
            f"theta={theta_est:6.2f}  acc={theta_acc:6.2f}  "
            f"gyro={gyro_rate:7.2f}  bias={bias_est:7.3f}  u={u:6.2f}"
        )

except KeyboardInterrupt:
    left_motor.power_command = 0.0
    right_motor.power_command = 0.0
    print("\nStopped.")
