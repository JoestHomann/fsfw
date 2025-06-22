from tmtccmd.logging import get_console_logger
from tmtccmd.tm import Service8FsfwTm
from tmtccmd.util import ObjectIdDictT
from tmtccmd.util.tmtc_printer import FsfwTmTcPrinter

LOGGER = get_console_logger()


def handle_action_reply(
    raw_tm: bytes, printer: FsfwTmTcPrinter, obj_id_dict: ObjectIdDictT
):
    """Core Action reply handler
    :return:
    """
    tm_packet = Service8FsfwTm.unpack(raw_telemetry=raw_tm)
    printer.handle_long_tm_print(packet_if=tm_packet, info_if=tm_packet)
    object_id = obj_id_dict.get(tm_packet.source_object_id_as_bytes)
    custom_data = tm_packet.custom_data
    action_id = tm_packet.action_id
    generic_print_str = printer.generic_action_packet_tm_print(
        packet=tm_packet, obj_id=object_id
    )
    print(generic_print_str)
    printer.file_logger.info(generic_print_str)
