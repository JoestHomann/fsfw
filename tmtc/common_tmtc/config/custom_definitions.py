"""
@brief      This file transfers control of the custom definitions like modes to the user.
@details    Template configuration file. Copy this folder to the TMTC commander root and adapt
            it to your needs.
"""

import enum
from enum import auto


class CustomServiceList(enum.IntEnum):
    pass


class SerialConfig(enum.Enum):
    SERIAL_PORT = auto()
    SERIAL_BAUD_RATE = auto()
    SERIAL_TIMEOUT = auto()
    SERIAL_COMM_TYPE = auto()


class EthernetConfig(enum.Enum):
    SEND_ADDRESS = auto()
    RECV_ADDRESS = auto()
