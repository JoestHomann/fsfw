"""
@brief      This file transfers control of the object IDs to the user.
@details    Template configuration file. Copy this folder to the TMTC commander root and adapt
            it to your needs.
"""
import os

from tmtccmd.fsfw import parse_fsfw_objects_csv
from tmtccmd.logging import get_console_logger
from tmtccmd.util.obj_id import ObjectIdDictT

LOGGER = get_console_logger()
DEFAULT_OBJECTS_CSV_PATH = "config/objects.csv"
__OBJECT_ID_DICT = None

PUS_SERVICE_17_ID = bytes([0x53, 0x00, 0x00, 0x17])
TEST_DEVICE_0_ID = bytes([0x44, 0x01, 0xAF, 0xFE])
TEST_DEVICE_1_ID = bytes([0x44, 0x02, 0xAF, 0xFE])
ASSEMBLY_ID = bytes([0x41, 0x00, 0xCA, 0xFE])


def get_object_ids() -> ObjectIdDictT:
    global __OBJECT_ID_DICT
    if not os.path.exists(DEFAULT_OBJECTS_CSV_PATH):
        LOGGER.warning(f"No Objects CSV file found at {DEFAULT_OBJECTS_CSV_PATH}")
    if __OBJECT_ID_DICT is None:
        if os.path.exists(DEFAULT_OBJECTS_CSV_PATH):
            __OBJECT_ID_DICT = parse_fsfw_objects_csv(csv_file=DEFAULT_OBJECTS_CSV_PATH)
        else:
            __OBJECT_ID_DICT = dict()
    return __OBJECT_ID_DICT
