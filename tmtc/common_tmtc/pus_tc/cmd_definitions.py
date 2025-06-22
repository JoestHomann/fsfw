from common_tmtc.pus_tc.service_20_parameters import add_param_cmds
from common_tmtc.pus_tc.service_3_housekeeping import add_hk_cmds
from tmtccmd.config import TmtcDefinitionWrapper
from common_tmtc.pus_tc.pus_11_tc_sched import add_tc_sched_cmds
from tmtccmd.config.globals import get_default_tmtc_defs
from tmtccmd.config.tmtc import OpCodeEntry


def common_fsfw_service_op_code_dict() -> TmtcDefinitionWrapper:
    def_wrapper = get_default_tmtc_defs()
    op_code_entry = OpCodeEntry()
    op_code_entry.add(keys="test", info="Mode CMD Test")
    op_code_entry.add(
        keys=["0", "asm_to_normal"], info="Command test assembly to normal mode"
    )
    def_wrapper.add_service(
        "200", info="PUS Service 200 Mode MGMT", op_code_entry=op_code_entry
    )

    op_code_entry = OpCodeEntry()
    op_code_entry.add(keys="ping", info="Send ping command")
    def_wrapper.add_service(
        "17", info="PUS Service 17 Ping", op_code_entry=op_code_entry
    )
    add_tc_sched_cmds(def_wrapper)
    add_param_cmds(def_wrapper)
    add_hk_cmds(def_wrapper)

    return def_wrapper
