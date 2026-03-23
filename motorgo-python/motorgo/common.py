# Collection of common tools used across all motor controller classes

import struct
import time


def crc16(data: bytes, polynomial: int = 0x1021, init_crc: int = 0xFFFF) -> int:
    crc = init_crc
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ polynomial
            else:
                crc <<= 1
            crc &= 0xFFFF  # Ensure CRC remains 16-bit
    return crc
