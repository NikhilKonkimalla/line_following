# Messages that are sent to and received from the Plink
import struct
import time
from abc import ABC, abstractmethod

from .brushed_motor_channel import MotorChannel


class Message:
    """A base class for messages that are sent to and received from the Plink."""

    SIZE = None
    TYPE = None


class MessageToPeri(Message, ABC):
    """A class representing a message that is sent to the peripheral device.

    The struct contains:
    - message_type (int)
    - data (bytes)
    """

    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def get_packed_struct(self) -> bytes:
        pass


class MessageFromPeri(Message):
    """A class representing a message that is received from the peripheral device.

    The struct contains:
    - valid (bool)
    - data (bytes)
    """

    @abstractmethod
    def __init__(self, data: bytes):
        self.data = data

    @abstractmethod
    def decode(self):
        pass

    @abstractmethod
    def __str__(self):
        pass


class InvalidToPeri(MessageToPeri):
    """A class representing an invalid message that is sent to the peripheral device.

    The struct contains:
    - message_type (int)
    """

    SIZE = 1
    TYPE = 0x00

    def __init__(self):
        pass

    def get_packed_struct(self, output_size=None) -> bytes:
        """
        Pack the structure data into bytes for transmission.
        """
        packed = struct.pack("<B", self.TYPE)

        assert len(packed) == self.SIZE

        if output_size is not None and output_size > len(packed):
            return packed + b"\x00" * (output_size - len(packed))

        return packed


class InvalidFromPeri(MessageFromPeri):
    """A class representing an invalid message that is received from the peripheral device.

    The struct contains:
    - valid (bool)
    """

    SIZE = 1
    TYPE = 0x00

    def __init__(self, data: bytes = None):
        super().__init__(data)

    def decode(self):
        pass

    def __str__(self):
        return f"Invalid Message"


class InitToPeri(MessageToPeri):
    """A class representing the data that is sent to the motor controller to initialize the motor controller

    The struct contains:
    - message_type (int)
    - Target Frequency (float)
    """

    SIZE = 25
    TYPE = 0x01

    def __init__(
        self,
        target_frequency: float,
        power_supply_voltage: float,
        channel_1_voltage_limit: float,
        channel_2_voltage_limit: float,
        channel_3_voltage_limit: float,
        channel_4_voltage_limit: float,
    ):
        self.target_frequency = target_frequency
        self.power_supply_voltage = power_supply_voltage
        self.channel_1_voltage_limit = channel_1_voltage_limit
        self.channel_2_voltage_limit = channel_2_voltage_limit
        self.channel_3_voltage_limit = channel_3_voltage_limit
        self.channel_4_voltage_limit = channel_4_voltage_limit

    def get_packed_struct(self, output_size=None) -> bytes:
        """
        Pack the structure data into bytes for transmission.
        """
        packed = struct.pack(
            "<B6f",
            self.TYPE,
            self.target_frequency,
            self.power_supply_voltage,
            self.channel_1_voltage_limit,
            self.channel_2_voltage_limit,
            self.channel_3_voltage_limit,
            self.channel_4_voltage_limit,
        )

        assert len(packed) == self.SIZE

        if output_size is not None and output_size > len(packed):
            return packed + b"\x00" * (output_size - len(packed))

        return packed


class InitFromPeri(MessageFromPeri):
    """A class representing the data that is received from the motor controller to initialize the motor controller

    The struct contains:
    - valid (bool)
    - board id (int 16 bit)
    - firmware version (int 8 bit)
    """

    SIZE = 5
    # Type 0x01
    TYPE = 0x01

    def __init__(self, data: bytes):
        super().__init__(data)

        self.valid = False
        self.board_id = None
        self.version_hash = None

    def decode(self):
        data = bytearray(self.data)[: self.SIZE]

        # Unpack two ints from the data
        message_type, board_id, version_hash = struct.unpack("<B2H", data)

        self.valid = message_type == self.TYPE

        if self.valid:
            self.board_id = board_id
            self.version_hash = version_hash

    def __str__(self):
        return f"Board ID: {self.board_id}, Firmware Version: {self.firmware_version}"


class DataToPeri(MessageToPeri):
    BUFFER_OUT_SIZE = 25
    TYPE = 0x02

    def __init__(
        self,
        channel1: MotorChannel,
        channel2: MotorChannel,
        channel3: MotorChannel,
        channel4: MotorChannel,
    ):
        """
        Initialize the output structure with motor channels' commands and modes.
        """

        self.channel_1_command = channel1.command
        self.channel_2_command = channel2.command
        self.channel_3_command = channel3.command
        self.channel_4_command = channel4.command

        self.channel_1_control_mode = channel1.control_mode
        self.channel_2_control_mode = channel2.control_mode
        self.channel_3_control_mode = channel3.control_mode
        self.channel_4_control_mode = channel4.control_mode

        self.channel_1_brake_mode = channel1.brake_mode
        self.channel_2_brake_mode = channel2.brake_mode
        self.channel_3_brake_mode = channel3.brake_mode
        self.channel_4_brake_mode = channel4.brake_mode

    def get_packed_struct(self, output_size=None) -> bytes:
        """
        Pack the structure data into bytes for transmission.
        """
        packed = struct.pack(
            "<B4f8B",
            self.TYPE,
            self.channel_1_command,
            self.channel_2_command,
            self.channel_3_command,
            self.channel_4_command,
            self.channel_1_control_mode,
            self.channel_2_control_mode,
            self.channel_3_control_mode,
            self.channel_4_control_mode,
            self.channel_1_brake_mode,
            self.channel_2_brake_mode,
            self.channel_3_brake_mode,
            self.channel_4_brake_mode,
        )

        assert len(packed) == self.BUFFER_OUT_SIZE

        if output_size is not None and output_size > len(packed):
            return packed + b"\x00" * (output_size - len(packed))

        return packed

    def __str__(self) -> str:
        """
        Return a string representation of the output structure.
        """
        return (
            f"OutputStruct:\n"
            f"Message Type: {self.TYPE}\n"
            f"Channel 1 Command: {self.channel_1_command}\n"
            f"Channel 2 Command: {self.channel_2_command}\n"
            f"Channel 3 Command: {self.channel_3_command}\n"
            f"Channel 4 Command: {self.channel_4_command}\n"
            f"Channel 1 Control Mode: {self.channel_1_control_mode}\n"
            f"Channel 2 Control Mode: {self.channel_2_control_mode}\n"
            f"Channel 3 Control Mode: {self.channel_3_control_mode}\n"
            f"Channel 4 Control Mode: {self.channel_4_control_mode}\n"
            f"Channel 1 Brake Mode: {self.channel_1_brake_mode}\n"
            f"Channel 2 Brake Mode: {self.channel_2_brake_mode}\n"
            f"Channel 3 Brake Mode: {self.channel_3_brake_mode}\n"
            f"Channel 4 Brake Mode: {self.channel_4_brake_mode}\n"
        )


class DataFromPeri(MessageFromPeri):
    BUFFER_IN_SIZE = 69
    TYPE = 0x02

    def __init__(self, data: bytes = None):
        """
        Initialize the input structure and decode data if provided.
        """
        super().__init__(data)

        self.valid = False
        self.channel_1_pos = 0
        self.channel_1_vel = 0
        self.channel_2_pos = 0
        self.channel_2_vel = 0
        self.channel_3_pos = 0
        self.channel_3_vel = 0
        self.channel_4_pos = 0
        self.channel_4_vel = 0

        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0

        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0

        self.mag_x = 0
        self.mag_y = 0
        self.mag_z = 0

    def decode(self):
        """
        Decode the input data into the structure fields.
        """
        data = bytearray(self.data)

        unpacked_data = struct.unpack_from("<B17f", data)

        self.valid = unpacked_data[0] == self.TYPE
        self.channel_1_pos = unpacked_data[1]
        self.channel_1_vel = unpacked_data[2]
        self.channel_2_pos = unpacked_data[3]
        self.channel_2_vel = unpacked_data[4]
        self.channel_3_pos = unpacked_data[5]
        self.channel_3_vel = unpacked_data[6]
        self.channel_4_pos = unpacked_data[7]
        self.channel_4_vel = unpacked_data[8]

        self.gyro_x = unpacked_data[9]
        self.gyro_y = unpacked_data[10]
        self.gyro_z = unpacked_data[11]
        self.accel_x = unpacked_data[12]
        self.accel_y = unpacked_data[13]
        self.accel_z = unpacked_data[14]
        self.mag_x = unpacked_data[15]
        self.mag_y = unpacked_data[16]
        self.mag_z = unpacked_data[17]

    def __str__(self) -> str:
        """
        Return a string representation of the input structure.
        """
        return (
            f"InputStruct:\n"
            f"Valid: {self.valid}\n"
            f"Channel 1 Position: {self.channel_1_pos}\n"
            f"Channel 1 Velocity: {self.channel_1_vel}\n"
            f"Channel 2 Position: {self.channel_2_pos}\n"
            f"Channel 2 Velocity: {self.channel_2_vel}\n"
            f"Channel 3 Position: {self.channel_3_pos}\n"
            f"Channel 3 Velocity: {self.channel_3_vel}\n"
            f"Channel 4 Position: {self.channel_4_pos}\n"
            f"Channel 4 Velocity: {self.channel_4_vel}\n"
        )


class PIDToPeri(MessageToPeri):
    BUFFER_OUT_SIZE = 22
    TYPE = 0x03

    def __init__(
        self,
        channel: int,
        p: float,
        i: float,
        d: float,
        output_ramp: float = 10000,
        lpf: float = 0.0,
    ):
        """
        Initialize the output structure with PID constants.
        """
        # Convert channel from int to a byte
        # Confirm that the channel is within the valid range
        assert 1 <= channel <= 4
        self.channel = channel
        self.p = p
        self.i = i
        self.d = d
        self.output_ramp = output_ramp
        self.lpf = lpf

    def get_packed_struct(self, output_size=None) -> bytes:
        """
        Pack the structure data into bytes for transmission.
        """
        packed = struct.pack(
            "<2B5f",
            self.TYPE,
            self.channel,
            self.p,
            self.i,
            self.d,
            self.output_ramp,
            self.lpf,
        )

        assert len(packed) == self.BUFFER_OUT_SIZE

        if output_size is not None and output_size > len(packed):
            return packed + b"\x00" * (output_size - len(packed))

        return packed
