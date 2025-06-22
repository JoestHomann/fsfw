"""
@brief      This file transfers control of TM parsing to the user
@details    Template configuration file. Copy this folder to the TMTC commander root and adapt
            it to your needs.
"""
from spacepackets.ecss.pus_17_test import Service17Tm
import spacepackets.ecss.pus_17_test as pus17
from spacepackets.ecss.tm import PusTelemetry
from spacepackets.util import PrintFormats

from tmtccmd.logging.pus import RawTmtcTimedLogWrapper
from tmtccmd.pus import VerificationWrapper
from tmtccmd.tm.pus_2_rawcmd import Service2Tm
from tmtccmd.tm.pus_20_fsfw_parameters import Service20FsfwTm
from tmtccmd.tm.pus_200_fsfw_modes import Service200FsfwTm
from tmtccmd.logging import get_console_logger

from common_tmtc.config.object_ids import get_object_ids
from common_tmtc.pus_tm.action_reply_handling import handle_action_reply
from common_tmtc.pus_tm.event_handler import handle_event_packet
from common_tmtc.pus_tm.verification_handler import handle_service_1_fsfw_packet
from common_tmtc.pus_tm.hk_handling import handle_hk_packet
from tmtccmd.util.tmtc_printer import FsfwTmTcPrinter

LOGGER = get_console_logger()


def pus_factory_hook(
    wrapper: VerificationWrapper,
    packet: bytes,
    printer: FsfwTmTcPrinter,
    raw_logger: RawTmtcTimedLogWrapper,
):
    if len(packet) < 8:
        LOGGER.warning("Detected packet shorter than 8 bytes!")
        return
    try:
        tm_packet = PusTelemetry.unpack(packet)
    except ValueError:
        LOGGER.warning("Could not generate PUS TM object from raw data")
        LOGGER.warning(f"Raw Packet: [{packet.hex(sep=',')}], REPR: {packet!r}")
        return
    service = tm_packet.service
    file_logger = printer.file_logger
    obj_id_dict = get_object_ids()
    dedicated_handler = True
    if service == 1:
        handle_service_1_fsfw_packet(wrapper=wrapper, raw_tm=packet)
    elif service == 2:
        tm_packet = Service2Tm.unpack(packet)
        dedicated_handler = False
    elif service == 3:
        handle_hk_packet(printer=printer, raw_tm=packet, obj_id_dict=obj_id_dict)
    elif service == 8:
        handle_action_reply(raw_tm=packet, printer=printer, obj_id_dict=obj_id_dict)
    elif service == 5:
        handle_event_packet(raw_tm=packet, printer=printer, file_logger=file_logger)
    elif service == 17:
        tm_packet = Service17Tm.unpack(raw_telemetry=packet)
        if tm_packet.subservice == pus17.Subservices.TM_REPLY:
            wrapper.dlog("Received Ping Reply TM[17,2]")
        dedicated_handler = True
    elif service == 20:
        tm_packet = Service20FsfwTm.unpack(raw_telemetry=packet)
        dedicated_handler = False
    elif service == 200:
        tm_packet = Service200FsfwTm.unpack(raw_telemetry=packet)
        dedicated_handler = False
    else:
        LOGGER.info(f"The service {service} is not implemented in Telemetry Factory")
        tm_packet = PusTelemetry.unpack(raw_telemetry=packet)
        tm_packet.print_source_data(PrintFormats.HEX)
        dedicated_handler = True
    if not dedicated_handler and tm_packet is not None:
        printer.handle_long_tm_print(packet_if=tm_packet, info_if=tm_packet)
    raw_logger.log_tm(tm_packet)
