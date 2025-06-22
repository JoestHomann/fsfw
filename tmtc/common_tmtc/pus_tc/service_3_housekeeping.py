from datetime import timedelta

from spacepackets.ecss.tc import PusTelecommand
from spacepackets.ecss import PusServices
from tmtccmd.config import TmtcDefinitionWrapper, OpCodeEntry
from tmtccmd.tc.pus_200_fsfw_modes import pack_mode_data, Modes
from tmtccmd.tc.pus_20_params import (
    pack_boolean_parameter_app_data,
    pack_fsfw_load_param_cmd,
)
from tmtccmd.tc.pus_3_fsfw_hk import make_sid, generate_one_hk_command
import tmtccmd.tc.pus_3_fsfw_hk as srv3
from tmtccmd.tc.pus_8_funccmd import make_fsfw_action_cmd

from common_tmtc.config.object_ids import TEST_DEVICE_0_ID, TEST_DEVICE_1_ID


# Set IDs
from tmtccmd.tc.queue import DefaultPusQueueHelper

TEST_SET_ID = 0

# Action IDs
TEST_NOTIFICATION_ACTION_ID = 3

# Parameters
PARAM_ACTIVATE_CHANGING_DATASETS = 4


def add_hk_cmds(defs: TmtcDefinitionWrapper):
    op_code_entry = OpCodeEntry()
    op_code_entry.add(keys=["0", "test"], info="Generic Test")
    defs.add_service(
        name=str(PusServices.S3_HOUSEKEEPING.value),
        info="PUS Service 3 Housekeeping",
        op_code_entry=op_code_entry,
    )


def pack_service_3_commands_into(q: DefaultPusQueueHelper, op_code: str):
    current_ssc = 3000
    device_idx = 0
    if device_idx == 0:
        object_id = TEST_DEVICE_0_ID
    else:
        object_id = TEST_DEVICE_1_ID

    if op_code == "0":
        # This will pack all the tests
        pack_service_3_test_info(
            q=q,
            object_id=object_id,
            device_idx=device_idx,
        )
    elif op_code == "1":
        # Extremely simple, generate one HK packet
        pack_gen_one_hk_command(
            q=q,
            device_idx=device_idx,
            object_id=object_id,
        )
    elif op_code == "2":
        # Housekeeping basic test
        pack_housekeeping_basic_test(q=q, object_id=object_id)
    elif op_code == "3":
        # Notification demo
        pack_notification_basic_test(q=q, object_id=object_id)


def pack_service_3_test_info(
    q: DefaultPusQueueHelper, device_idx: int, object_id: bytearray
):
    q.add_log_cmd("Service 3 (Housekeeping Service): All tests")
    pack_gen_one_hk_command(q=q, device_idx=device_idx, object_id=object_id)
    pack_housekeeping_basic_test(q=q, object_id=object_id)
    pack_notification_basic_test(
        q=q,
        object_id=object_id,
        enable_normal_mode=False,
    )


def pack_gen_one_hk_command(
    q: DefaultPusQueueHelper, device_idx: int, object_id: bytearray
):
    test_sid = make_sid(object_id=object_id, set_id=TEST_SET_ID)
    q.add_log_cmd(
        f"Service 3 Test: Generate one test set packet for test device {device_idx}"
    )
    q.add_pus_tc(generate_one_hk_command(sid=test_sid))


def pack_housekeeping_basic_test(
    q: DefaultPusQueueHelper,
    object_id: bytearray,
    enable_normal_mode: bool = True,
):
    """
    This basic test will request one HK packet, then it will enable periodic packets and listen
    to the periodic packets for a few seconds. After that, HK packets will be disabled again.
    """
    test_sid = make_sid(object_id=object_id, set_id=TEST_SET_ID)
    # Enable changing datasets via parameter service (Service 20)
    q.add_log_cmd("Service 3 Test: Performing basic HK tests")

    if enable_normal_mode:
        # Set mode normal so that sets are changed/read regularly
        q.add_log_cmd("Service 3 Test: Set Normal Mode")
        mode_data = pack_mode_data(object_id, Modes.NORMAL, 0)
        q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))

    q.add_log_cmd("Enabling changing datasets")
    app_data = pack_boolean_parameter_app_data(
        object_id=object_id,
        domain_id=0,
        unique_id=PARAM_ACTIVATE_CHANGING_DATASETS,
        parameter=True,
    )
    q.add_pus_tc(pack_fsfw_load_param_cmd(app_data=app_data))

    # Enable periodic reporting
    q.add_log_cmd("Enabling periodic thermal sensor packet generation: ")
    q.add_pus_tc(
        PusTelecommand(
            service=3,
            subservice=srv3.Subservices.TC_ENABLE_PERIODIC_HK_GEN,
            app_data=test_sid,
        )
    )
    q.add_wait(timedelta(seconds=2.0))

    # Disable periodic reporting
    q.add_log_cmd("Disabling periodic thermal sensor packet generation: ")
    q.add_pus_tc(
        PusTelecommand(
            service=3,
            subservice=srv3.Subservices.TC_DISABLE_PERIODIC_HK_GEN,
            app_data=test_sid,
        )
    )

    # Disable changing datasets via parameter service (Service 20)
    q.add_log_cmd("Disabling changing datasets")
    app_data = pack_boolean_parameter_app_data(
        object_id=object_id,
        domain_id=0,
        unique_id=PARAM_ACTIVATE_CHANGING_DATASETS,
        parameter=False,
    )
    q.add_pus_tc(pack_fsfw_load_param_cmd(app_data=app_data))


def pack_notification_basic_test(
    q: DefaultPusQueueHelper,
    object_id: bytearray,
    enable_normal_mode: bool = True,
):
    q.add_log_cmd("Service 3 Test: Performing notification tests")

    if enable_normal_mode:
        # Set mode normal so that sets are changed/read regularly
        q.add_log_cmd("Service 3 Test: Set Normal Mode")
        mode_data = pack_mode_data(object_id, Modes.NORMAL, 0)
        q.add_pus_tc(PusTelecommand(service=200, subservice=1, app_data=mode_data))

    q.add_log_cmd("Triggering notification")
    q.add_pus_tc(
        make_fsfw_action_cmd(object_id=object_id, action_id=TEST_NOTIFICATION_ACTION_ID)
    )
