import struct
import time
from datetime import timedelta

from spacepackets.ecss.tc import PusTelecommand

from tmtccmd.config import TmtcDefinitionWrapper
from tmtccmd.config.tmtc import OpCodeEntry
from tmtccmd.pus.pus_17_test import pack_service_17_ping_command
from tmtccmd.pus.pus_11_tc_sched import TypeOfTimeWindow, Subservices, TcSchedReqId
from tmtccmd.tc.pus_11_tc_sched import (
    generate_enable_tc_sched_cmd,
    generate_disable_tc_sched_cmd,
    generate_reset_tc_sched_cmd,
    pack_time_tagged_tc_app_data,
)
from tmtccmd.tc.queue import DefaultPusQueueHelper


class OpCodes:
    ENABLE = ["0", "enable"]
    DISABLE = ["1", "disable"]
    RESET = ["2", "reset"]

    TEST_INSERT = ["12", "test-insert"]
    TEST_DELETE = ["13", "test-del"]
    TEST_RESET = ["14", "test-clear"]


class Info:
    ENABLE = "Enable TC scheduling"
    DISABLE = "Disable TC scheduling"
    RESET = "Reset TC scheduling"
    TEST_INSERT = "Test TC scheduling insertion"
    TEST_DELETE = "Test TC scheduling deletion"
    TEST_RESET = "Test TC scheduling reset command"


def add_tc_sched_cmds(cmd_dict: TmtcDefinitionWrapper):
    op_code_entry = OpCodeEntry()
    op_code_entry.add(keys=OpCodes.ENABLE, info=Info.ENABLE)
    op_code_entry.add(keys=OpCodes.DISABLE, info=Info.DISABLE)
    op_code_entry.add(keys=OpCodes.RESET, info=Info.RESET)
    op_code_entry.add(keys=OpCodes.TEST_INSERT, info=Info.TEST_INSERT)
    op_code_entry.add(keys=OpCodes.TEST_DELETE, info=Info.TEST_DELETE)
    op_code_entry.add(keys=OpCodes.TEST_RESET, info=Info.TEST_RESET)
    cmd_dict.add_service(
        "11", info="PUS Service 11 TC Scheduling", op_code_entry=op_code_entry
    )


def __generic_pack_three_time_tagged_cmds(q: DefaultPusQueueHelper):
    q.add_log_cmd("Testing Time-Tagged Command insertion")
    q.add_pus_tc(generate_enable_tc_sched_cmd())
    current_time = int(round(time.time()))

    # these TC[17,1] (ping commands) shall be inserted
    ping_tcs = [
        pack_service_17_ping_command(),
        pack_service_17_ping_command(),
        pack_service_17_ping_command(),
    ]
    for idx, tc in enumerate(ping_tcs):
        release_time = current_time + (idx + 2) * 5
        q.add_pus_tc(
            PusTelecommand(
                service=11,
                subservice=Subservices.TC_INSERT,
                app_data=pack_time_tagged_tc_app_data(
                    struct.pack("!I", release_time), tc
                ),
            )
        )
    q.add_wait(timedelta(seconds=25.0))


def pack_service_11_commands(op_code: str, q: DefaultPusQueueHelper):
    if op_code in OpCodes.TEST_INSERT:
        q.add_log_cmd("Testing Time-Tagged Command deletion")
        __generic_pack_three_time_tagged_cmds(q=q)
    if op_code in OpCodes.TEST_DELETE:
        current_time = int(round(time.time()))
        tc_to_schedule = pack_service_17_ping_command()
        q.add_pus_tc(
            PusTelecommand(
                service=11,
                subservice=Subservices.TC_INSERT,
                app_data=pack_time_tagged_tc_app_data(
                    struct.pack("!I", current_time + 20), tc_to_schedule
                ),
            )
        )
        q.add_pus_tc(
            PusTelecommand(
                service=11,
                subservice=Subservices.TC_DELETE,
                app_data=pack_delete_corresponding_tc_app_data(tc=tc_to_schedule),
            )
        )
    if op_code in OpCodes.ENABLE:
        q.add_log_cmd("Enabling TC scheduler")
        q.add_pus_tc(generate_enable_tc_sched_cmd())
    if op_code in OpCodes.DISABLE:
        q.add_log_cmd("Disabling TC scheduler")
        q.add_pus_tc(generate_disable_tc_sched_cmd())
    if op_code in OpCodes.RESET:
        q.add_log_cmd("Reset TC scheduler")
        q.add_pus_tc(generate_reset_tc_sched_cmd())
    if op_code in OpCodes.TEST_RESET:
        q.add_log_cmd("Testing Reset command")
        __generic_pack_three_time_tagged_cmds(q)
        q.add_pus_tc(generate_reset_tc_sched_cmd())
        # a TC[11,5] for 3rd inserted ping TC
        # TODO: This should be an independent test
        # a TC[11,6] for some other previously inserted TCs
        # service_11_6_tc = build_filter_delete_tc(TypeOfTimeWindow.FROM_TIMETAG_TO_TIMETAG,
        #   current_time+45, current_time+55)
        # tc_queue.appendleft(service_11_6_tc.pack_command_tuple())

        # TODO: This should be an independent test
        # a TC[11,7] for another previously inserted TC
        # service_11_7_tc = build_corresponding_timeshift_tc(30, ping_tc_4)
        # tc_queue.appendleft(service_11_7_tc.pack_command_tuple())

        # TODO: This should be an independent test
        # a TC[11,8] with offset time of 20s
        # service_11_8_tc = build_filter_timeshift_tc(
        #      20,
        #     TypeOfTimeWindow.FROM_TIMETAG_TO_TIMETAG,
        #     current_time + 45,
        #     current_time + 55,
        # )
        # tc_queue.appendleft((service_11_8_tc.pack_command_tuple()))


def pack_delete_corresponding_tc_app_data(tc: PusTelecommand) -> bytes:
    return TcSchedReqId.build_from_tc(tc).pack()


def build_filter_delete_tc(
    time_window_type: TypeOfTimeWindow, *timestamps: int
) -> PusTelecommand:
    app_data = bytearray()
    app_data.extend(struct.pack("!I", int(time_window_type)))

    if time_window_type != TypeOfTimeWindow.SELECT_ALL:
        for timestamp in timestamps:
            app_data.extend(struct.pack("!I", timestamp))

    return PusTelecommand(service=11, subservice=6, app_data=app_data)


def pack_corresponding_timeshift_app_data(
    time_delta: bytes, tc: PusTelecommand, ssc: int
) -> bytes:
    req_id = TcSchedReqId.build_from_tc(tc)
    app_data = bytearray()
    app_data.extend(time_delta)
    app_data.extend(req_id.pack())
    return app_data


def build_filter_timeshift_tc(
    time_offset: int, time_window_type: TypeOfTimeWindow, *timestamps: int
) -> PusTelecommand:
    app_data = bytearray()
    app_data.extend(struct.pack("!I", time_offset))
    app_data.extend(struct.pack("!I", int(time_window_type)))

    if time_window_type != TypeOfTimeWindow.SELECT_ALL:
        for timestamp in timestamps:
            app_data.extend(struct.pack("!I", timestamp))

    return PusTelecommand(service=11, subservice=8, app_data=app_data)


# waits for a specified amount of seconds and prints ". . ." for each second
def wait_seconds(t: int):
    print("Waiting: ", end="")
    for x in range(t):
        time.sleep(1)
        print(". ", end="")

    print("")
