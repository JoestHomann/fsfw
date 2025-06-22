import os
from tmtccmd.fsfw import parse_fsfw_returnvalues_csv, RetvalDictT
from tmtccmd.logging import get_console_logger

DEFAULT_RETVAL_CSV_NAME = "config/returnvalues.csv"
__RETVAL_DICT = None
LOGGER = get_console_logger()


def get_retval_dict() -> RetvalDictT:
    global __RETVAL_DICT
    if __RETVAL_DICT is None:
        if os.path.exists(DEFAULT_RETVAL_CSV_NAME):
            __RETVAL_DICT = parse_fsfw_returnvalues_csv(
                csv_file=DEFAULT_RETVAL_CSV_NAME
            )
        else:
            LOGGER.warning(
                f"No Return Value CSV file found at {DEFAULT_RETVAL_CSV_NAME}"
            )
            __RETVAL_DICT = dict()
    return __RETVAL_DICT
