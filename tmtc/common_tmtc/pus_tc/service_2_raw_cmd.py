# -*- coding: utf-8 -*-
"""
@file   tmtcc_tc_service_2_raw_cmd.py
@brief  PUS Service 2: Device Access, native low-level commanding
@author R. Mueller
@date   01.11.2019
"""
import struct

from spacepackets.ecss.tc import PusTelecommand

from tmtccmd.tc.pus_200_fsfw_modes import Modes, pack_mode_data

from common_tmtc.pus_tc import command_data as cmd_data
from common_tmtc.config.object_ids import TEST_DEVICE_0_ID
from tmtccmd.tc.queue import DefaultPusQueueHelper


def pack_service_2_commands_into(q: DefaultPusQueueHelper, op_code: str):
    if op_code == "0":
        pack_generic_service_2_test_into(0, q)
    else:
        print(f"pack_service_2_test: Operation code {op_code} unknown!")


def pack_generic_service_2_test_into(init_ssc: int, q: DefaultPusQueueHelper) -> int:
    new_ssc = init_ssc
    object_id = TEST_DEVICE_0_ID  # dummy device
    # Set Raw Mode
    q.add_log_cmd("Testing Service 2: Setting Raw Mode")
    mode_data = pack_mode_data(object_id, Modes.RAW, 0)
    q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))
    # toggle wiretapping raw
    q.add_log_cmd("Testing Service 2: Toggling Wiretapping Raw")
    wiretapping_toggle_data = pack_wiretapping_mode(object_id, 1)
    q.add_pus_tc(
        PusTelecommand(service=2, subservice=129, app_data=wiretapping_toggle_data)
    )
    # send raw command, wiretapping should be returned via TM[2,130] and TC[2,131]
    q.add_log_cmd("Testing Service 2: Sending Raw Command")
    raw_command = cmd_data.TEST_COMMAND_0
    raw_data = object_id + raw_command
    q.add_pus_tc(PusTelecommand(service=2, subservice=128, app_data=raw_data))
    # toggle wiretapping off
    q.add_log_cmd("Testing Service 2: Toggle Wiretapping Off")
    wiretapping_toggle_data = pack_wiretapping_mode(object_id, 0)
    q.add_pus_tc(
        PusTelecommand(service=2, subservice=129, app_data=wiretapping_toggle_data)
    )
    # send raw command which should be returned via TM[2,130]
    q.add_log_cmd("Testing Service 2: Send second raw command")
    q.add_pus_tc(PusTelecommand(service=2, subservice=128, app_data=raw_data))
    # Set mode off
    q.add_log_cmd("Testing Service 2: Setting Off Mode")
    mode_data = pack_mode_data(object_id, Modes.OFF, 0)
    q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))
    return new_ssc


# wiretappingMode = 0: MODE_OFF, wiretappingMode = 1: MODE_RAW
def pack_wiretapping_mode(object_id, wiretapping_mode_):
    wiretapping_mode = struct.pack(
        ">B", wiretapping_mode_
    )  # MODE_OFF : 0x00, MODE_RAW: 0x01
    wiretapping_toggle_data = object_id + wiretapping_mode
    return wiretapping_toggle_data
