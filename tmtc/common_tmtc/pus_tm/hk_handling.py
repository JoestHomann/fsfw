"""This file transfers control of housekeeping handling (PUS service 3) to the developer
"""
import struct
from typing import Tuple

from tmtccmd.tm import Service3FsfwTm
from tmtccmd.tm.pus_3_hk_base import HkContentType, Service3Base
from tmtccmd.logging import get_console_logger

from common_tmtc.config.object_ids import TEST_DEVICE_0_ID, TEST_DEVICE_1_ID
from tmtccmd.util import ObjectIdDictT, ObjectIdU32
from tmtccmd.util.tmtc_printer import FsfwTmTcPrinter

LOGGER = get_console_logger()


def handle_hk_packet(
    raw_tm: bytes,
    obj_id_dict: ObjectIdDictT,
    printer: FsfwTmTcPrinter,
):
    tm_packet = Service3FsfwTm.unpack(raw_telemetry=raw_tm, custom_hk_handling=False)
    named_obj_id = obj_id_dict.get(tm_packet.object_id.as_bytes)
    if named_obj_id is None:
        named_obj_id = tm_packet.object_id
    if tm_packet.subservice == 25 or tm_packet.subservice == 26:
        hk_data = tm_packet.tm_data[8:]
        printer.generic_hk_tm_print(
            content_type=HkContentType.HK,
            object_id=named_obj_id,
            set_id=tm_packet.set_id,
            hk_data=hk_data,
        )
        handle_regular_hk_print(
            printer=printer,
            object_id=named_obj_id,
            hk_packet=tm_packet,
            hk_data=hk_data,
        )
    if tm_packet.subservice == 10 or tm_packet.subservice == 12:
        LOGGER.warning("HK definitions printout not implemented yet")


def handle_regular_hk_print(
    printer: FsfwTmTcPrinter,
    object_id: ObjectIdU32,
    hk_packet: Service3Base,
    hk_data: bytes,
):
    if object_id == TEST_DEVICE_0_ID or object_id == TEST_DEVICE_1_ID:
        handle_test_set_deserialization(hk_data=hk_data)


def handle_test_set_deserialization(
    hk_data: bytes,
) -> Tuple[list, list, bytearray, int]:
    header_list = []
    content_list = []
    validity_buffer = bytearray()
    # uint8 (1) + uint32_t (4) + float vector with 3 entries (12) + validity buffer (1)
    if len(hk_data) < 18:
        LOGGER.warning("Invalid HK data format for test set reply!")
        return header_list, content_list, validity_buffer, 0
    uint8_value = struct.unpack("!B", hk_data[0:1])[0]
    uint32_value = struct.unpack("!I", hk_data[1:5])[0]
    float_value_1 = struct.unpack("!f", hk_data[5:9])[0]
    float_value_2 = struct.unpack("!f", hk_data[9:13])[0]
    float_value_3 = struct.unpack("!f", hk_data[13:17])[0]
    validity_buffer.append(hk_data[17])
    header_list.append("uint8 value")
    header_list.append("uint32 value")
    header_list.append("float vec value 1")
    header_list.append("float vec value 2")
    header_list.append("float vec value 3")

    content_list.append(uint8_value)
    content_list.append(uint32_value)
    content_list.append(float_value_1)
    content_list.append(float_value_2)
    content_list.append(float_value_3)
    return header_list, content_list, validity_buffer, 3
