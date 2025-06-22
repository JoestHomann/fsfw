"""
@brief      This file transfers control of custom mode handling to the user.
@details    Template configuration file. Copy this folder to the TMTC commander root and adapt
            it to your needs.
"""
import sys

from tmtccmd.core.backend import TmTcHandler
from tmtccmd.utility.logger import get_logger

LOGGER = get_logger()


def perform_mode_operation_user(tmtc_backend: TmTcHandler, mode: int):
    """
    Custom modes can be implemented here
    """
    LOGGER.error(f"Unknown mode {mode}, Configuration error !")
    sys.exit()
