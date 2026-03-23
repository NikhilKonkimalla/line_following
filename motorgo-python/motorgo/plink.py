import atexit
import struct
import threading
import time
from collections import deque

import spidev
from gpiozero import DigitalInputDevice, DigitalOutputDevice

from .brushed_motor_channel import BrakeMode, ControlMode, MotorChannel
from .common import crc16
from .imu import IMU
from .message_parser import MessageParser
from .messages import (
    DataFromPeri,
    DataToPeri,
    InitFromPeri,
    InitToPeri,
    InvalidFromPeri,
    InvalidToPeri,
    MessageFromPeri,
    MessageToPeri,
    PIDToPeri,
)
from .version import __version__


class Plink:

    SUPPORTED_BOARD_IDS = [0x03]

    def __init__(self, frequency: int = 200, timeout: float = 1.0):
        """
        Initialize the Plink communication object with motor channels and communication settings.
        """
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # Open bus 0, device (CS) 0
        self.spi.mode = 3
        self.spi.max_speed_hz = 7_200_000  # Set SPI speed

        self.last_message_time = None

        self.channel1 = MotorChannel()
        self.channel2 = MotorChannel()
        self.channel3 = MotorChannel()
        self.channel4 = MotorChannel()

        self.frequency = frequency
        self.timeout = timeout

        self.running = False
        self.connected = False

        # Add an extra 4 bytes at the end for padding
        # esp32 slave has a bug that requires this
        self.transfer_size = 76

        self.imu = IMU(self.frequency)

        self.data_ready_pin = DigitalInputDevice(25)
        self.reset_pin = DigitalOutputDevice(22, active_high=False, initial_value=False)

        self.config_message_queue = deque()

        self.__power_supply_voltage = 10.0

    def reset(self):
        """
        Reset the Plink by toggling the reset pin.
        """
        self.reset_pin.blink(on_time=0.05, off_time=0.05, n=1)

    def connect(self):
        """
        Start the communication thread.
        """
        atexit.register(self.shutdown)  # Register cleanup function

        if self.__power_supply_voltage is None:
            raise ValueError("Voltage limit must be set before connecting")

        # Reset the Plink
        self.reset()

        self.connected = self.initialize_comms()

        # If connected, start the comms thread
        if self.connected:
            print("Connected to Plink")

            # Start the communication thread
            self.running = True
            self.thread = threading.Thread(target=self.comms_thread)
            self.thread.daemon = True  # Set the thread as a daemon thread
            self.thread.start()

        else:
            print("Not starting connection to protect MotorGo")

    def update_motor_states(self, response: DataFromPeri):
        """
        Update the motor states based on the input structure data.

        Args:
            response (InputStruct): The input structure containing the motor states.
        """
        # Update the motor states
        self.channel1.update_position(response.channel_1_pos)
        self.channel1.update_velocity(response.channel_1_vel)

        self.channel2.update_position(response.channel_2_pos)
        self.channel2.update_velocity(response.channel_2_vel)

        self.channel3.update_position(response.channel_3_pos)
        self.channel3.update_velocity(response.channel_3_vel)

        self.channel4.update_position(response.channel_4_pos)
        self.channel4.update_velocity(response.channel_4_vel)

        # Update the IMU data
        self.imu.update(
            response.gyro_x,
            response.gyro_y,
            response.gyro_z,
            response.accel_x,
            response.accel_y,
            response.accel_z,
            response.mag_x,
            response.mag_y,
            response.mag_z,
        )

    def initialize_comms(self):
        """Prepare and send the initialization data."""

        print("Connecting to Plink...")

        # First, send an invalid message to reset the SPI state
        data = InvalidToPeri()

        # Transfer data until a valid response is received
        message = None
        while not message:
            message = self.transfer(data)

        # Prepare data actual initialization data
        data = InitToPeri(
            self.frequency,
            self.__power_supply_voltage,
            5.0,
            5.0,
            5.0,
            5.0,
        )

        initialized = False

        while not initialized:
            response = self.transfer(data)

            # Check that the response is of the correct type
            if isinstance(response, InitFromPeri):
                initialized = True

        valid = True

        # TODO: Confirm that the response is correct
        expected_version = crc16(__version__.encode())
        if response.version_hash != expected_version:
            print("Invalid version received")
            valid = False

        if response.board_id not in self.SUPPORTED_BOARD_IDS:
            print("Unsupported board ID received")
            valid = False

        return valid

    def transfer(self, message: MessageToPeri, timeout: float = 1.0) -> MessageFromPeri:
        """
        Prepare and send data, then receive and process the response.
        """

        # Wait for data ready pin to go high, or timeout
        # If the data ready pin is not high, return an invalid response
        if not self.data_ready_pin.wait_for_active(timeout=timeout):
            # Failed transfer, return None to indicate failure
            return None

            # Send data and receive response (Mock response for now)
        response = MessageParser().parse(
            self.spi.xfer2(message.get_packed_struct(self.transfer_size))
        )

        # wait for data ready pin to go low, or timeout
        if not self.data_ready_pin.wait_for_inactive(timeout=timeout):
            # Failed transfer, return None to indicate failure
            return None

        # Update last message time, if valid message received
        if not isinstance(response, InvalidFromPeri):
            self.last_message_time = time.time()

        return response

    def update_motorgo(
        self,
    ):
        """
        Prepare and send data, then receive and process the response.
        """

        # Preapre any config update messages
        self.prepare_config_update_messages()

        # Check for any messages in the queue
        if self.config_message_queue:
            message = self.config_message_queue.popleft()
        else:
            # Normal case, prepare motor command data
            # Prepare data from current state
            message = DataToPeri(
                self.channel1, self.channel2, self.channel3, self.channel4
            )

        response = self.transfer(message)

        # Update the motor states
        if isinstance(response, DataFromPeri):
            self.update_motor_states(response)
            self.last_message_time = time.time()

        elif isinstance(response, InitFromPeri):
            print("Received initialization response, re-doing initialization")
            self.initialize_comms()

        elif isinstance(response, InvalidFromPeri):
            print("Invalid response received")

    def prepare_config_update_messages(self):
        """
        Prepare messages for any config updates to be sent to the MotorGo board.
        """

        # Check all motor channels for pending updates
        for i, channel in enumerate(
            [self.channel1, self.channel2, self.channel3, self.channel4]
        ):
            channel_number = i + 1
            if channel._pid_update_ready:

                new_params = channel._get_velocity_gain_update()

                # Prepare and add the PID update message to the queue
                self.config_message_queue.append(PIDToPeri(channel_number, *new_params))

    def comms_thread(self):
        """
        Communication thread to handle periodic data transfer.
        """
        try:
            while self.running:

                if self.connected:
                    self.update_motorgo()

                    # Check for response timeout
                    if self.last_message_time is not None:
                        if time.time() - self.last_message_time > self.timeout:
                            print("MotorGo response timeout")
                            self.connected = False
                else:
                    # If not connected, attempt to re-initialize
                    print("Attempting to re-initialize connection")
                    self.connected = self.initialize_comms()

        except KeyboardInterrupt:
            # Handle keyboard interrupt (Ctrl+C)
            self.shutdown()

        finally:
            # Ensure cleanup code runs when the thread is stopped
            self.shutdown()

    def shutdown(self):
        """
        Clean up resources and perform shutdown tasks.
        """
        self.running = False

        self.reset()

        print("Disconnecting from Plink ...")
        # Close the SPI connection
        # self.spi.close()
