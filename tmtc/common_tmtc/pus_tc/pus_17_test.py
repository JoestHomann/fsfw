from tmtccmd.pus.pus_17_test import (
    pack_service_17_ping_command,
)
from tmtccmd.tc.queue import DefaultPusQueueHelper
from tmtccmd.logging import get_console_logger

LOGGER = get_console_logger()


def pack_service_17_commands(op_code: str, q: DefaultPusQueueHelper):
    if op_code in ["0", "ping"]:
        q.add_pus_tc(pack_service_17_ping_command())
    else:
        LOGGER.warning(f"Invalid op code {op_code}")
