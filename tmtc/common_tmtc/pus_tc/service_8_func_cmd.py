from spacepackets.ecss.tc import PusTelecommand
from tmtccmd.tc.pus_200_fsfw_modes import pack_mode_data, Modes

import common_tmtc.pus_tc.command_data as cmd_data


from common_tmtc.config.object_ids import TEST_DEVICE_0_ID
from tmtccmd.tc.queue import DefaultPusQueueHelper


def pack_service_8_commands_into(q: DefaultPusQueueHelper, op_code: str):
    if op_code == "0":
        pack_generic_service_8_test_into(q=q)
    else:
        print(f"pack_service_8_test: Operation code {op_code} unknown!")


def pack_generic_service_8_test_into(q: DefaultPusQueueHelper):
    q.add_log_cmd("Testing Service 8")
    object_id = TEST_DEVICE_0_ID

    # set mode on
    q.add_log_cmd("Testing Service 8: Set On Mode")
    mode_data = pack_mode_data(object_id, Modes.ON, 0)
    q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))

    # set mode normal
    q.add_log_cmd("Testing Service 8: Set Normal Mode")
    mode_data = pack_mode_data(object_id, Modes.NORMAL, 0)
    q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))

    # Direct command which triggers completion reply
    q.add_log_cmd("Testing Service 8: Trigger Step and Completion Reply")
    action_id = cmd_data.TEST_COMMAND_0
    direct_command = object_id + action_id
    q.add_pus_tc(PusTelecommand(service=8, subservice=128, app_data=direct_command))

    # Direct command which triggers _tm_data reply
    q.add_log_cmd("Testing Service 8: Trigger Data Reply")
    action_id = cmd_data.TEST_COMMAND_1
    command_param1 = cmd_data.TEST_COMMAND_1_PARAM_1
    command_param2 = cmd_data.TEST_COMMAND_1_PARAM_2
    direct_command = object_id + action_id + command_param1 + command_param2
    q.add_pus_tc(PusTelecommand(service=8, subservice=128, app_data=direct_command))

    # Set mode off
    q.add_log_cmd("Testing Service 8: Set Off Mode")
    mode_data = pack_mode_data(object_id, Modes.OFF, 0)
    q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))
