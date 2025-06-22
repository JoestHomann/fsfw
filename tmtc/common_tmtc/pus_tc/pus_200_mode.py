# -*- coding: utf-8 -*-
from spacepackets.ecss.tc import PusTelecommand

from tmtccmd.tc import QueueHelperBase
from tmtccmd.tc.pus_200_fsfw_modes import pack_mode_data, Modes

from common_tmtc.config.object_ids import TEST_DEVICE_0_ID, ASSEMBLY_ID
from tmtccmd.tc.queue import DefaultPusQueueHelper


def pack_service_200_commands_into(q: DefaultPusQueueHelper, op_code: str):
    if op_code == "test":
        pack_service_200_test_into(q)
    elif op_code == "asm_to_normal" or op_code == "0":
        # Set Normal mode
        q.add_log_cmd("Testing Service 200: Set Mode Normal")
        # Command assembly to normal, submode 1 for dual mode,
        mode_data = pack_mode_data(ASSEMBLY_ID, Modes.NORMAL, 1)
        q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))


def pack_service_200_test_into(q: DefaultPusQueueHelper):
    q.add_log_cmd("Testing Service 200")
    # Object ID: DUMMY Device
    object_id = TEST_DEVICE_0_ID
    # Set On Mode
    q.add_log_cmd("Testing Service 200: Set Mode On")
    mode_data = pack_mode_data(object_id, Modes.ON, 0)
    q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))
    # Set Normal mode
    q.add_log_cmd("Testing Service 200: Set Mode Normal")
    mode_data = pack_mode_data(object_id, Modes.NORMAL, 0)
    q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))
    # Set Raw Mode
    q.add_log_cmd("Testing Service 200: Set Mode Raw")
    mode_data = pack_mode_data(object_id, Modes.RAW, 0)
    q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))
    # Set Off Mode
    q.add_log_cmd("Testing Service 200: Set Mode Off")
    mode_data = pack_mode_data(object_id, Modes.OFF, 0)
    q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))
