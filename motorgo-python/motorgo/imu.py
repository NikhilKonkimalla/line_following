import threading
import time

import imufusion
import numpy as np


class IMU:
    def __init__(self, frequency, mag_enabled=False):
        """
        Initialize the IMU with default values.
        """
        self.gyro_state = np.array([0.0, 0.0, 0.0])
        self.accel_state = np.array([0.0, 0.0, 0.0])
        self.mag_state = np.array([0.0, 0.0, 0.0])

        self.frequency = frequency

        self.lock = threading.Lock()

        self.ahrs = imufusion.Ahrs()
        self.ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NWU,  # convention
            2.0,  # gain
            2000,  # gyroscope range
            0,  # acceleration rejection
            0,  # magnetic rejection
            self.frequency * 5,  # recovery trigger period = 5 seconds
        )

        self.offset = imufusion.Offset(self.frequency)
        self.last_update_time = None

        self.mag_enabled = mag_enabled

    def update(
        self,
        gyro_x: float,
        gyro_y: float,
        gyro_z: float,
        accel_x: float,
        accel_y: float,
        accel_z: float,
        mag_x: float,
        mag_y: float,
        mag_z: float,
    ):
        """
        Update the IMU data with the provided list.
        """

        self._update(
            np.array([gyro_x, gyro_y, gyro_z]),
            np.array([accel_x, accel_y, accel_z]),
            np.array([mag_x, mag_y, mag_z]),
        )

    def _update(self, gyro: np.ndarray, accel: np.ndarray, mag: np.ndarray):

        gyro = self.offset.update(gyro)
        gyro = np.degrees(gyro)  # Convert to degrees/s

        # Update internal state
        with self.lock:
            self.gyro_state = gyro
            self.accel_state = accel
            self.mag_state = mag

        #  Get the time delta, just use 1/frequency if no last update time
        if self.last_update_time is not None:
            dt = time.time() - self.last_update_time
        else:
            dt = 1 / self.frequency
        self.last_update_time = time.time()

        if self.mag_enabled:
            self.ahrs.update(gyro, accel, mag, dt)
        else:
            self.ahrs.update_no_magnetometer(gyro, accel, dt)

    @property
    def gyro(self) -> np.ndarray:
        """
        Get the gyroscope data as an np.ndarray.
        """
        with self.lock:
            return self.gyro_state

    @property
    def accel(self) -> np.ndarray:
        """
        Get the accelerometer data as a np.ndarray.
        """

        with self.lock:
            return self.accel_state

    @property
    def mag(self) -> np.ndarray:
        """
        Get the magnetometer data as a np.ndarray.
        """
        with self.lock:
            return self.mag_state

    @property
    def gravity_vector(self) -> np.ndarray:
        """
        Get the gravity vector calculated by the AHRS.
        """

        return self.ahrs.gravity

    @property
    def quaternion(self) -> np.ndarray:
        """
        Get the quaternion calculated by the AHRS.
        """

        return self.ahrs.quaternion

    def __str__(self) -> str:
        """
        Return a string representation of the IMU data.
        """
        return (
            f"IMU:\n"
            f"Gyro X: {self.gyro_x}\n"
            f"Gyro Y: {self.gyro_y}\n"
            f"Gyro Z: {self.gyro_z}\n"
            f"Accel X: {self.accel_x}\n"
            f"Accel Y: {self.accel_y}\n"
            f"Accel Z: {self.accel_z}\n"
            f"Mag X: {self.mag_x}\n"
            f"Mag Y: {self.mag_y}\n"
            f"Mag Z: {self.mag_z}\n"
        )
