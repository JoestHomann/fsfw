import struct

from spacepackets.ecss.tc import PusTelecommand
from spacepackets.ecss import PusServices

from tmtccmd.config import TmtcDefinitionWrapper, OpCodeEntry
from tmtccmd.tc.pus_20_params import pack_type_and_matrix_data, pack_parameter_id
from tmtccmd.tc.pus_200_fsfw_modes import pack_mode_data, Modes
from tmtccmd.logging import get_console_logger

from common_tmtc.config.object_ids import TEST_DEVICE_0_ID
from tmtccmd.tc.queue import DefaultPusQueueHelper

LOGGER = get_console_logger()


def add_param_cmds(defs: TmtcDefinitionWrapper):
    op_code_entry = OpCodeEntry()
    op_code_entry.add(keys=["0", "test"], info="Generic Test")
    defs.add_service(
        name=str(PusServices.S20_PARAMETER.value),
        info="PUS Service 20 Parameters",
        op_code_entry=op_code_entry,
    )


def pack_service20_commands_into(q: DefaultPusQueueHelper, op_code: str):
    if op_code == "0":
        pack_service20_test_into(q=q)


def pack_service20_test_into(q: DefaultPusQueueHelper, called_externally: bool = False):
    if called_externally is False:
        q.add_log_cmd("Testing Service 20")
    object_id = TEST_DEVICE_0_ID

    # set mode on
    q.add_log_cmd("Testing Service 20: Set Mode On")
    mode_data = pack_mode_data(object_id, Modes.ON, 0)
    q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))
    # set mode normal
    q.add_log_cmd("Testing Service 20: Set Normal Mode")
    mode_data = pack_mode_data(object_id, Modes.NORMAL, 0)
    q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))

    load_param_0_simple_test_commands(q)
    load_param_1_simple_test_commands(q)
    load_param_2_simple_test_commands(q)


def load_param_0_simple_test_commands(q: DefaultPusQueueHelper):
    object_id = TEST_DEVICE_0_ID
    parameter_id_0 = pack_parameter_id(domain_id=0, unique_id=0, linear_index=0)
    # test checking Load for uint32_t
    q.add_log_cmd("Testing Service 20: Load uint32_t")
    type_and_matrix_data = pack_type_and_matrix_data(3, 14, 1, 1)
    parameter_data = struct.pack("!I", 42)
    payload = object_id + parameter_id_0 + type_and_matrix_data + parameter_data
    q.add_pus_tc(PusTelecommand(service=20, subservice=128, app_data=payload))

    # test checking Dump for uint32_t
    q.add_log_cmd("Testing Service 20: Dump uint32_t")
    payload = object_id + parameter_id_0
    q.add_pus_tc(PusTelecommand(service=20, subservice=129, app_data=payload))


def load_param_1_simple_test_commands(q: DefaultPusQueueHelper):
    object_id = TEST_DEVICE_0_ID
    parameter_id_1 = pack_parameter_id(domain_id=0, unique_id=1, linear_index=0)
    # test checking Load for int32_t
    q.add_log_cmd("Testing Service 20: Load int32_t")
    type_and_matrix_data = pack_type_and_matrix_data(4, 14, 1, 1)
    parameter_data = struct.pack("!i", -42)
    payload = object_id + parameter_id_1 + type_and_matrix_data + parameter_data
    q.add_pus_tc(PusTelecommand(service=20, subservice=128, app_data=payload))

    # test checking Dump for int32_t
    q.add_log_cmd("Testing Service 20: Dump int32_t")
    payload = object_id + parameter_id_1
    q.add_pus_tc(PusTelecommand(service=20, subservice=129, app_data=payload))


def load_param_2_simple_test_commands(q: DefaultPusQueueHelper):
    object_id = TEST_DEVICE_0_ID
    parameter_id_2 = pack_parameter_id(domain_id=0, unique_id=2, linear_index=0)
    # test checking Load for float
    q.add_log_cmd("Testing Service 20: Load float")
    type_and_matrix_data = pack_type_and_matrix_data(ptc=5, pfc=1, rows=1, columns=3)
    parameter_data = struct.pack("!fff", 4.2, -4.2, 49)
    payload = object_id + parameter_id_2 + type_and_matrix_data + parameter_data
    q.add_pus_tc(PusTelecommand(service=20, subservice=128, app_data=payload))

    # test checking Dump for float
    # Skip dump for now, still not properly implemented
    # tc_queue.appendleft((QueueCommands.PRINT, "Testing Service 20: Dump float"))
    # payload = object_id + parameter_id_2
    # command = PusTelecommand(service=20, subservice=129, ssc=2060, app_data=payload)
    # tc_queue.appendleft(command.pack_command_tuple())
