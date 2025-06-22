import logging
import sys
from pathlib import Path
from typing import cast

from spacepackets import SpacePacket, SpacePacketHeader
from spacepackets.ccsds import SPACE_PACKET_HEADER_SIZE
from spacepackets.cfdp import (
    TransmissionMode,
    ChecksumType,
    ConditionCode,
    PduHolder,
    DirectiveType,
    PduFactory,
    PduType,
)
from spacepackets.cfdp.pdu import MetadataPdu, FileDataPdu
from tmtccmd.cfdp import (
    RemoteEntityCfg,
    LocalEntityCfg,
    CfdpUserBase,
    IndicationCfg,
    TransactionId,
)
from tmtccmd.cfdp.defs import CfdpRequestType
from tmtccmd.cfdp.handler import CfdpInCcsdsHandler
from tmtccmd.cfdp.mib import DefaultFaultHandlerBase
from tmtccmd.cfdp.user import (
    FileSegmentRecvdParams,
    MetadataRecvParams,
    TransactionFinishedParams,
)
from tmtccmd.config.args import ProcedureParamsWrapper
from tmtccmd.logging import get_current_time_string
from tmtccmd.pus.pus_11_tc_sched import Subservices as Pus11Subservices
from tmtccmd.tc.queue import DefaultPusQueueHelper
from tmtccmd.util import FileSeqCountProvider, PusFileSeqCountProvider
from tmtccmd.util.tmtc_printer import FsfwTmTcPrinter

try:
    import spacepackets
except ImportError as error:
    print(error)
    print("Python tmtccmd module could not be imported. Make sure it is installed")
    sys.exit(1)

try:
    import tmtccmd
except ImportError as error:
    print(error)
    print("Python tmtccmd module could not be imported. Make sure it is installed")
    sys.exit(1)

from spacepackets.ecss import PusVerificator, PusTelecommand, PusServices

from common_tmtc.pus_tc.pus_11_tc_sched import pack_service_11_commands
from common_tmtc.pus_tc.pus_17_test import pack_service_17_commands
from common_tmtc.pus_tc.pus_200_mode import pack_service_200_commands_into
from common_tmtc.pus_tc.service_20_parameters import pack_service20_commands_into
from common_tmtc.pus_tc.service_2_raw_cmd import pack_service_2_commands_into
from common_tmtc.pus_tc.service_3_housekeeping import pack_service_3_commands_into
from common_tmtc.pus_tc.service_8_func_cmd import pack_service_8_commands_into
from examples.tmtcc import (
    EXAMPLE_PUS_APID,
    EXAMPLE_CFDP_APID,
    CFDP_LOCAL_ENTITY_ID,
    CFDP_REMOTE_ENTITY_ID,
)
from tmtccmd import (
    TcHandlerBase,
    get_console_logger,
    TmTcCfgHookBase,
    CcsdsTmtcBackend,
)
from tmtccmd.pus import VerificationWrapper
from tmtccmd.tc import (
    ProcedureWrapper,
    FeedWrapper,
    TcProcedureType,
    TcQueueEntryType,
    SendCbParams,
)
from tmtccmd.tc.pus_5_event import pack_generic_service_5_test_into
from tmtccmd.tm import SpecificApidHandlerBase, CcsdsTmHandler
from tmtccmd.logging.pus import RawTmtcTimedLogWrapper
from tmtccmd.config import (
    CoreServiceList,
    SetupWrapper,
    SetupParams,
    PreArgsParsingWrapper,
    params_to_procedure_conversion,
)
from common_tmtc.pus_tm.factory_hook import pus_factory_hook


LOGGER = get_console_logger()


class ExampleCfdpFaultHandler(DefaultFaultHandlerBase):
    def notice_of_suspension_cb(self, cond: ConditionCode):
        pass

    def notice_of_cancellation_cb(self, cond: ConditionCode):
        pass

    def abandoned_cb(self, cond: ConditionCode):
        pass

    def ignore_cb(self, cond: ConditionCode):
        pass


class ExampleCfdpUser(CfdpUserBase):
    def transaction_indication(self, transaction_id: TransactionId):
        LOGGER.info(f"CFDP User: Start of File {transaction_id}")

    def eof_sent_indication(self, transaction_id: TransactionId):
        LOGGER.info(f"CFDP User: EOF sent for {transaction_id}")

    def transaction_finished_indication(self, params: TransactionFinishedParams):
        LOGGER.info(f"CFDP User: {params.transaction_id} finished")

    def metadata_recv_indication(self, params: MetadataRecvParams):
        pass

    def file_segment_recv_indication(self, params: FileSegmentRecvdParams):
        pass

    def report_indication(self, transaction_id: TransactionId, status_report: any):
        pass

    def suspended_indication(
        self, transaction_id: TransactionId, cond_code: ConditionCode
    ):
        pass

    def resumed_indication(self, transaction_id: TransactionId, progress: int):
        pass

    def fault_indication(
        self, transaction_id: TransactionId, cond_code: ConditionCode, progress: int
    ):
        pass

    def abandoned_indication(
        self, transaction_id: TransactionId, cond_code: ConditionCode, progress: int
    ):
        pass

    def eof_recv_indication(self, transaction_id: TransactionId):
        pass


class CfdpInCcsdsWrapper(SpecificApidHandlerBase):
    def __init__(self, cfdp_in_ccsds_handler: CfdpInCcsdsHandler):
        super().__init__(EXAMPLE_CFDP_APID, None)
        self.handler = cfdp_in_ccsds_handler

    def handle_tm(self, packet: bytes, _user_args: any):
        # Ignore the space packet header. Its only purpose is to use the same protocol and
        # have a seaprate APID for space packets. If this function is called, the APID is correct.
        pdu = packet[SPACE_PACKET_HEADER_SIZE:]
        pdu_base = PduFactory.from_raw(pdu)
        if pdu_base.pdu_type == PduType.FILE_DATA:
            LOGGER.info("Received File Data PDU TM")
        else:
            if pdu_base.directive_type == DirectiveType.FINISHED_PDU:
                LOGGER.info(f"Received Finished PDU TM")
            else:
                LOGGER.info(
                    f"Received File Directive PDU with type {pdu_base.directive_type!r} TM"
                )
        self.handler.pass_pdu_packet(pdu_base)


class PusHandler(SpecificApidHandlerBase):
    def __init__(
        self,
        wrapper: VerificationWrapper,
        printer: FsfwTmTcPrinter,
        raw_logger: RawTmtcTimedLogWrapper,
    ):
        super().__init__(EXAMPLE_PUS_APID, None)
        self.printer = printer
        self.verif_wrapper = wrapper
        self.raw_logger = raw_logger

    def handle_tm(self, packet: bytes, _user_args: any):
        pus_factory_hook(
            packet=packet,
            wrapper=self.verif_wrapper,
            raw_logger=self.raw_logger,
            printer=self.printer,
        )


class TcHandler(TcHandlerBase):
    def __init__(
        self,
        seq_count_provider: FileSeqCountProvider,
        cfdp_in_ccsds_wrapper: CfdpInCcsdsWrapper,
        pus_verificator: PusVerificator,
        file_logger: logging.Logger,
        raw_logger: RawTmtcTimedLogWrapper,
    ):
        super().__init__()
        self.cfdp_handler_started = False
        self.cfdp_dest_id = CFDP_REMOTE_ENTITY_ID
        self.seq_count_provider = seq_count_provider
        self.pus_verificator = pus_verificator
        self.file_logger = file_logger
        self.raw_logger = raw_logger
        self.queue_helper = DefaultPusQueueHelper(
            queue_wrapper=None,
            pus_apid=EXAMPLE_PUS_APID,
            seq_cnt_provider=seq_count_provider,
            pus_verificator=pus_verificator,
        )
        self.cfdp_in_ccsds_wrapper = cfdp_in_ccsds_wrapper

    def cfdp_done(self) -> bool:
        return not self.cfdp_in_ccsds_wrapper.handler.put_request_pending()

    def feed_cb(self, info: ProcedureWrapper, wrapper: FeedWrapper):
        self.queue_helper.queue_wrapper = wrapper.queue_wrapper
        if info.proc_type == TcProcedureType.DEFAULT:
            self.handle_default_procedure(info)
        elif info.proc_type == TcProcedureType.CFDP:
            self.handle_cfdp_procedure(info)

    def handle_default_procedure(self, info: ProcedureWrapper):
        def_proc = info.to_def_procedure()
        service = def_proc.service
        op_code = def_proc.op_code
        if service == CoreServiceList.SERVICE_2.value:
            return pack_service_2_commands_into(op_code=op_code, q=self.queue_helper)
        if service == CoreServiceList.SERVICE_3.value:
            return pack_service_3_commands_into(op_code=op_code, q=self.queue_helper)
        if service == CoreServiceList.SERVICE_5.value:
            return pack_generic_service_5_test_into(q=self.queue_helper)
        if service == CoreServiceList.SERVICE_8.value:
            return pack_service_8_commands_into(op_code=op_code, q=self.queue_helper)
        if service == CoreServiceList.SERVICE_11.value:
            return pack_service_11_commands(op_code=op_code, q=self.queue_helper)
        if service == CoreServiceList.SERVICE_17.value:
            return pack_service_17_commands(op_code=op_code, q=self.queue_helper)
        if service == CoreServiceList.SERVICE_20.value:
            return pack_service20_commands_into(q=self.queue_helper, op_code=op_code)
        if service == CoreServiceList.SERVICE_200.value:
            return pack_service_200_commands_into(q=self.queue_helper, op_code=op_code)
        LOGGER.warning("Invalid Service !")

    def handle_cfdp_procedure(self, info: ProcedureWrapper):
        cfdp_procedure = info.to_cfdp_procedure()
        if cfdp_procedure.cfdp_request_type == CfdpRequestType.PUT:
            if (
                not self.cfdp_in_ccsds_wrapper.handler.put_request_pending()
                and not self.cfdp_handler_started
            ):
                put_req = cfdp_procedure.request_wrapper.to_put_request()
                put_req.cfg.destination_id = self.cfdp_dest_id
                LOGGER.info(
                    f"CFDP: Starting file put request with parameters:\n{put_req}"
                )
                self.cfdp_in_ccsds_wrapper.handler.cfdp_handler.put_request(put_req)
                self.cfdp_handler_started = True

                for source_pair, dest_pair in self.cfdp_in_ccsds_wrapper.handler:
                    pdu, sp = source_pair
                    pdu = cast(PduHolder, pdu)
                    if pdu.is_file_directive:
                        if pdu.pdu_directive_type == DirectiveType.METADATA_PDU:
                            metadata = pdu.to_metadata_pdu()
                            self.queue_helper.add_log_cmd(
                                f"CFDP Source: Sending Metadata PDU for file with size "
                                f"{metadata.file_size}"
                            )
                        elif pdu.pdu_directive_type == DirectiveType.EOF_PDU:
                            self.queue_helper.add_log_cmd(
                                f"CFDP Source: Sending EOF PDU"
                            )
                    else:
                        fd_pdu = pdu.to_file_data_pdu()
                        self.queue_helper.add_log_cmd(
                            f"CFDP Source: Sending File Data PDU for segment at offset "
                            f"{fd_pdu.offset} with length {len(fd_pdu.file_data)}"
                        )
                    self.queue_helper.add_ccsds_tc(sp)
                    self.cfdp_in_ccsds_wrapper.handler.confirm_source_packet_sent()
                self.cfdp_in_ccsds_wrapper.handler.source_handler.state_machine()

    def send_cb(self, params: SendCbParams):
        if params.entry.is_tc:
            if params.entry.entry_type == TcQueueEntryType.PUS_TC:
                self.handle_tc_send_cb(params)
            elif params.entry.entry_type == TcQueueEntryType.CCSDS_TC:
                cfdp_packet_in_ccsds = params.entry.to_space_packet_entry()
                params.com_if.send(cfdp_packet_in_ccsds.space_packet.pack())
        elif params.entry.entry_type == TcQueueEntryType.LOG:
            log_entry = params.entry.to_log_entry()
            LOGGER.info(log_entry.log_str)
            self.file_logger.info(log_entry.log_str)

    def handle_tc_send_cb(self, params: SendCbParams):
        pus_tc_wrapper = params.entry.to_pus_tc_entry()
        if (
            pus_tc_wrapper.pus_tc.service == PusServices.S11_TC_SCHED
            and pus_tc_wrapper.pus_tc.subservice == Pus11Subservices.TC_INSERT
        ):
            wrapped_tc = PusTelecommand.unpack(pus_tc_wrapper.pus_tc.app_data[4:])
            tc_info_string = f"Sending time-tagged command {wrapped_tc}"
            LOGGER.info(tc_info_string)
            self.file_logger.info(f"{get_current_time_string(True)}: {tc_info_string}")
        raw_tc = pus_tc_wrapper.pus_tc.pack()
        self.raw_logger.log_tc(pus_tc_wrapper.pus_tc)
        tc_info_string = f"Sending {pus_tc_wrapper.pus_tc}"
        LOGGER.info(tc_info_string)
        self.file_logger.info(f"{get_current_time_string(True)}: {tc_info_string}")
        params.com_if.send(raw_tc)

    def queue_finished_cb(self, info: ProcedureWrapper):
        if info is not None:
            if info.proc_type == TcQueueEntryType.PUS_TC:
                def_proc = info.to_def_procedure()
                LOGGER.info(
                    f"Finished queue for service {def_proc.service} and op code {def_proc.op_code}"
                )
            elif info.proc_type == TcProcedureType.CFDP:
                LOGGER.info(f"Finished CFDP queue")
                self.cfdp_sending_done = True


def setup_params(hook_obj: TmTcCfgHookBase) -> SetupWrapper:
    print(f"-- eive TMTC Commander --")
    print(f"-- spacepackets v{spacepackets.__version__} --")
    params = SetupParams()
    parser_wrapper = PreArgsParsingWrapper()
    parser_wrapper.create_default_parent_parser()
    parser_wrapper.create_default_parser()
    parser_wrapper.add_def_proc_and_cfdp_as_subparsers()
    post_arg_parsing_wrapper = parser_wrapper.parse(hook_obj)
    tmtccmd.init_printout(post_arg_parsing_wrapper.use_gui)
    use_prompts = not post_arg_parsing_wrapper.use_gui
    proc_param_wrapper = ProcedureParamsWrapper()
    if use_prompts:
        post_arg_parsing_wrapper.set_params_with_prompts(params, proc_param_wrapper)
    else:
        post_arg_parsing_wrapper.set_params_without_prompts(params, proc_param_wrapper)
    params.apid = EXAMPLE_PUS_APID
    setup_wrapper = SetupWrapper(
        hook_obj=hook_obj, setup_params=params, proc_param_wrapper=proc_param_wrapper
    )
    return setup_wrapper


def setup_cfdp_handler() -> CfdpInCcsdsWrapper:
    fh_base = ExampleCfdpFaultHandler()
    cfdp_cfg = LocalEntityCfg(
        local_entity_id=CFDP_LOCAL_ENTITY_ID,
        indication_cfg=IndicationCfg(),
        default_fault_handlers=fh_base,
    )
    remote_cfg = RemoteEntityCfg(
        closure_requested=False,
        entity_id=CFDP_REMOTE_ENTITY_ID,
        max_file_segment_len=1024,
        check_limit=None,
        crc_on_transmission=False,
        crc_type=ChecksumType.CRC_32,
        default_transmission_mode=TransmissionMode.UNACKNOWLEDGED,
    )
    cfdp_seq_count_provider = FileSeqCountProvider(
        max_bit_width=16, file_name=Path("seqcnt_cfdp_transaction.txt")
    )
    cfdp_ccsds_seq_count_provider = PusFileSeqCountProvider(
        file_name=Path("seqcnt_cfdp_ccsds_.txt")
    )
    cfdp_user = ExampleCfdpUser()
    cfdp_in_ccsds_handler = CfdpInCcsdsHandler(
        cfg=cfdp_cfg,
        remote_cfgs=[remote_cfg],
        ccsds_apid=EXAMPLE_CFDP_APID,
        ccsds_seq_cnt_provider=cfdp_ccsds_seq_count_provider,
        cfdp_seq_cnt_provider=cfdp_seq_count_provider,
        user=cfdp_user,
    )
    return CfdpInCcsdsWrapper(cfdp_in_ccsds_handler)


def setup_tmtc_handlers(
    verif_wrapper: VerificationWrapper,
    printer: FsfwTmTcPrinter,
    raw_logger: RawTmtcTimedLogWrapper,
) -> (CcsdsTmHandler, TcHandler):
    cfdp_in_ccsds_wrapper = setup_cfdp_handler()
    pus_handler = PusHandler(
        printer=printer, raw_logger=raw_logger, wrapper=verif_wrapper
    )
    ccsds_handler = CcsdsTmHandler(None)
    ccsds_handler.add_apid_handler(pus_handler)
    ccsds_handler.add_apid_handler(cfdp_in_ccsds_wrapper)
    tc_handler = TcHandler(
        file_logger=printer.file_logger,
        raw_logger=raw_logger,
        pus_verificator=verif_wrapper.pus_verificator,
        seq_count_provider=PusFileSeqCountProvider(
            file_name=Path("seqcnt_pus_ccsds.txt")
        ),
        cfdp_in_ccsds_wrapper=cfdp_in_ccsds_wrapper,
    )
    return ccsds_handler, tc_handler


def setup_backend(
    setup_wrapper: SetupWrapper,
    tc_handler: TcHandler,
    ccsds_handler: CcsdsTmHandler,
) -> CcsdsTmtcBackend:
    init_proc = params_to_procedure_conversion(setup_wrapper.proc_param_wrapper)
    tmtc_backend = tmtccmd.create_default_tmtc_backend(
        setup_wrapper=setup_wrapper,
        tm_handler=ccsds_handler,
        tc_handler=tc_handler,
        init_procedure=init_proc,
    )
    tmtccmd.start(tmtc_backend=tmtc_backend, hook_obj=setup_wrapper.hook_obj)
    return cast(CcsdsTmtcBackend, tmtc_backend)
