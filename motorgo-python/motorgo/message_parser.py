import struct
import time

from .messages import DataFromPeri, InitFromPeri, InvalidFromPeri, MessageFromPeri


class MessageParser:
    """A singleton class to parse messages from the Plink."""

    _instance = None
    AVAILABLE_MESSAGES = [InitFromPeri, DataFromPeri]

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(MessageParser, cls).__new__(cls)
            cls._instance.message_map = {
                message.TYPE: message for message in cls.AVAILABLE_MESSAGES
            }
        return cls._instance

    def parse(self, data: list[bytes]) -> MessageFromPeri:
        """Parse a message from the Plink, return the message object."""

        # Read the first byte to get the message type
        message_type = struct.unpack("<B", bytearray(data[0:1]))[0]

        # Get the message class from the message map
        message_class = self.message_map.get(message_type)

        if message_class is None:
            return InvalidFromPeri(data)

        # Create an instance of the message class
        message = message_class(data)

        # Decode the message
        message.decode()

        return message
