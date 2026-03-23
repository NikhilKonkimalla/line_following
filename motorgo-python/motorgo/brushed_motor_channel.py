import threading


# Enums to match C++ definitions
class ControlMode:
    VELOCITY = 2
    POWER = 1
    NONE = 0


class BrakeMode:
    BRAKE = 0
    COAST = 1


class MotorChannel:
    def __init__(self):
        """
        Initialize the MotorChannel with default control and brake modes.
        """
        self.lock = threading.Lock()

        self.control_mode = ControlMode.POWER
        self.brake_mode = BrakeMode.BRAKE

        # Variables to store current commands
        self._velocity_command = 0.0
        self._power_command = 0.0

        self._position = 0.0
        self._velocity = 0.0

        # PID parameters for velocity control
        self.velocity_p = 0.0
        self.velocity_i = 0.0
        self.velocity_d = 0.0
        self.velocity_output_ramp = 10000.0
        self.velocity_lpf = 0.0

        self._pid_update_ready = False

        self.motor_voltage_limit = 0.0

    @property
    def velocity_command(self) -> float:
        """
        Get the velocity command with thread-safe access.
        """
        with self.lock:
            return self._velocity_command

    @velocity_command.setter
    def velocity_command(self, value: float):
        """
        Set the velocity command with thread-safe access and mode check.
        """
        with self.lock:
            if self._control_mode != ControlMode.VELOCITY:
                print(
                    "Warning: Setting velocity command with control mode set to power."
                )
            self._velocity_command = value

    @property
    def power_command(self) -> float:
        """
        Get the power command with thread-safe access.
        """
        with self.lock:
            return self._power_command

    @power_command.setter
    def power_command(self, value: float):
        """
        Set the power command with thread-safe access and mode check.
        """
        with self.lock:
            if self._control_mode != ControlMode.POWER:
                print(
                    "Warning: Setting power command with control mode set to velocity."
                )
            self._power_command = value

    @property
    def control_mode(self) -> int:
        """
        Get the control mode with thread-safe access.
        """
        with self.lock:
            return self._control_mode

    @control_mode.setter
    def control_mode(self, value: int):
        """
        Set the control mode with thread-safe access.
        """
        with self.lock:
            self._control_mode = value

    @property
    def brake_mode(self) -> int:
        """
        Get the brake mode with thread-safe access.
        """
        with self.lock:
            return self._brake_mode

    @brake_mode.setter
    def brake_mode(self, value: int):
        """
        Set the brake mode with thread-safe access.
        """
        with self.lock:
            self._brake_mode = value

    @property
    def command(self) -> float:
        """
        Get the command based on the current control mode.
        """
        return (
            self.velocity_command
            if self.control_mode == ControlMode.VELOCITY
            else self.power_command
        )

    @property
    def position(self) -> float:
        """
        Get the position with thread-safe access.
        """
        with self.lock:
            return self._position

    def update_position(self, value: float):
        """
        Update the position with thread-safe access.
        """
        with self.lock:
            self._position = value

    @property
    def velocity(self) -> float:
        """
        Get the velocity with thread-safe access.
        """
        with self.lock:
            return self._velocity

    def update_velocity(self, value: float):
        """
        Update the velocity with thread-safe access.
        """
        with self.lock:
            self._velocity = value

    def set_velocity_pid_gains(
        self,
        p: float = None,
        i: float = None,
        d: float = None,
        output_ramp: float = None,
        lpf: float = None,
    ):
        """
        Set the PID gains for velocity control.
        """

        with self.lock:
            if p is not None:
                self.velocity_p = p
                self._pid_update_ready = True
            if i is not None:
                self.velocity_i = i
                self._pid_update_ready = True
            if d is not None:
                self.velocity_d = d
                self._pid_update_ready = True
            if output_ramp is not None:
                self.velocity_output_ramp = output_ramp
                self._pid_update_ready = True
            if lpf is not None:
                self.velocity_lpf = lpf
                self._pid_update_ready = True

    def _get_velocity_gain_update(self) -> tuple:
        """Returns the new PID gains and resets the flag.

        Returns:
            tuple: A tuple containing the PID gains.
        """

        if self._pid_update_ready:
            with self.lock:
                self._pid_update_ready = False
                return (
                    self.velocity_p,
                    self.velocity_i,
                    self.velocity_d,
                    self.velocity_output_ramp,
                    self.velocity_lpf,
                )

        return None
