import logging
import os.path
from datetime import datetime
from common_tmtc.config.object_ids import get_object_ids

from tmtccmd.tm import Service5Tm
from tmtccmd.logging import get_console_logger
from tmtccmd.fsfw import parse_fsfw_events_csv, EventDictT, EventInfo
from tmtccmd.util.tmtc_printer import FsfwTmTcPrinter

LOGGER = get_console_logger()
DEFAULT_EVENTS_CSV_PATH = "config/events.csv"
__EVENT_DICT = None


def get_event_dict() -> EventDictT:
    global __EVENT_DICT
    if __EVENT_DICT is None:
        if os.path.exists(DEFAULT_EVENTS_CSV_PATH):
            __EVENT_DICT = parse_fsfw_events_csv(DEFAULT_EVENTS_CSV_PATH)
        else:
            LOGGER.warning(f"No Event CSV file found at {DEFAULT_EVENTS_CSV_PATH}")
            __EVENT_DICT = dict()
    return __EVENT_DICT


def handle_event_packet(
    raw_tm: bytes, printer: FsfwTmTcPrinter, file_logger: logging.Logger
) -> str:
    tm = Service5Tm.unpack(raw_telemetry=raw_tm)
    printer.handle_long_tm_print(packet_if=tm, info_if=tm)
    additional_event_info = ""
    event_dict = get_event_dict()
    info = event_dict.get(tm.event_id)
    if info is None:
        LOGGER.warning(f"Event ID {tm.event_id} has no information")
        info = EventInfo()
        info.name = "Unknown event"
    obj_ids = get_object_ids()
    obj_id_obj = obj_ids.get(tm.reporter_id.as_bytes)
    if obj_id_obj is None:
        LOGGER.warning(f"Object ID 0x{tm.reporter_id.name} has no name")
        obj_name = tm.reporter_id.name
    else:
        obj_name = obj_id_obj.name
    generic_event_string = (
        f"Object {obj_name} generated Event {tm.event_id} | {info.name}"
    )
    if info.info != "":
        additional_event_info = (
            f"Additional info: {info.info} | P1: {tm.param_1} | P2: {tm.param_2}"
        )
    file_logger.info(
        f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}: {generic_event_string}"
    )
    LOGGER.info(generic_event_string)
    if additional_event_info != "":
        file_logger.info(additional_event_info)
        print(additional_event_info)
    return generic_event_string + " | " + additional_event_info
