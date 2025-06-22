from spacepackets.ecss.pus_1_verification import UnpackParams, Service1Tm

from tmtccmd.pus import VerificationWrapper
from tmtccmd.tm.pus_1_verification import Service1TmExtended, Service1FsfwWrapper
from tmtccmd.logging import get_console_logger
from common_tmtc.config.retvals import get_retval_dict

LOGGER = get_console_logger()


def handle_service_1_fsfw_packet(wrapper: VerificationWrapper, raw_tm: bytes):
    if wrapper.console_logger is None or wrapper.file_logger is None:
        raise ValueError(
            "Console logger or file logger not valid. Please set a valid one"
        )
    # Error code with length 2 is FSFW specific
    tm_packet = Service1Tm.unpack(data=raw_tm, params=UnpackParams(1, 2))
    fsfw_wrapper = Service1FsfwWrapper(tm_packet)
    res = wrapper.verificator.add_tm(tm_packet)
    if res is None:
        LOGGER.info(
            f"Received Verification TM[{tm_packet.service}, {tm_packet.subservice}] "
            f"with Request ID {tm_packet.tc_req_id.as_u32():#08x}"
        )
        LOGGER.warning(f"No matching telecommand found for {tm_packet.tc_req_id}")
    else:
        wrapper.log_to_console(tm_packet, res)
        wrapper.log_to_file(tm_packet, res)
    retval_dict = get_retval_dict()
    if tm_packet.has_failure_notice:
        retval_info = retval_dict.get(tm_packet.error_code.val)
        if retval_info is None:
            raw_err = tm_packet.error_code.val
            LOGGER.info(
                f"No returnvalue information found for error code  with subsystem ID"
                f" {(raw_err >> 8) & 0xff} and unique ID {raw_err & 0xff}"
            )
        else:
            retval_string = (
                f"Error Code information for code {tm_packet.error_code.val:#06x} | "
                f"Name: {retval_info.name} | Info: {retval_info.info}"
            )
            error_param_1_str = (
                f"Error Parameter 1: hex {fsfw_wrapper.error_param_1:#010x} "
                f"dec{fsfw_wrapper.error_param_1} "
            )
            error_param_2_str = (
                f"Error Parameter 2: hex {fsfw_wrapper.error_param_2:#010x} "
                f"dec {fsfw_wrapper.error_param_2}"
            )
            wrapper.dlog(retval_string)
            wrapper.dlog(error_param_1_str)
            wrapper.dlog(error_param_2_str)
