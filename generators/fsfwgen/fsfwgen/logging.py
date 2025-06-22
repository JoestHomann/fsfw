import logging
import sys
from typing import Optional
from colorlog import ColoredFormatter


FSFWGEN_LOGGER_NAME = "fsfwgen"
__CONSOLE_LOGGER_SET_UP: Optional[logging.Logger] = None


def get_console_logger() -> logging.Logger:
    global __CONSOLE_LOGGER_SET_UP
    """Get the global console logger instance. Error logs will still be saved to an error file
    """
    logger = logging.getLogger(FSFWGEN_LOGGER_NAME)
    if not __CONSOLE_LOGGER_SET_UP:
        __CONSOLE_LOGGER_SET_UP = True
        __setup_tmtc_console_logger()
    return logger


def init_console_logger(log_level: int = logging.INFO) -> logging.Logger:
    global __CONSOLE_LOGGER_SET_UP
    if not __CONSOLE_LOGGER_SET_UP:
        __CONSOLE_LOGGER_SET_UP = True
        return __setup_tmtc_console_logger(log_level=log_level)
    return get_console_logger()


def __setup_tmtc_console_logger(log_level: int = logging.INFO) -> logging.Logger:
    """Sets the LOGGER object which will be used globally. This needs to be called before
    using the logger.
    :return:    Returns the instance of the global logger
    """
    logger = logging.getLogger(FSFWGEN_LOGGER_NAME)
    # Use colorlog for now because it allows more flexibility and custom messages
    # for different levels
    set_up_colorlog_logger(logger=logger)
    logger.setLevel(level=log_level)
    # set_up_coloredlogs_logger(logger=logger)
    return logger


# Custom formatter. Allows different strings for info, error and debug output
class CustomTmtccmdFormatter(ColoredFormatter):
    def __init__(
        self, info_fmt: str, dbg_fmt: str, err_fmt: str, warn_fmt: str, datefmt=None
    ):
        self.err_fmt = err_fmt
        self.info_fmt = info_fmt
        self.dbg_fmt = dbg_fmt
        self.warn_fmt = warn_fmt
        super().__init__(fmt="%(levelno)d: %(msg)s", datefmt=datefmt, style="%")

    def format(self, record):

        # Save the original format configured by the user
        # when the logger formatter was instantiated
        format_orig = self._style._fmt

        # Replace the original format with one customized by logging level
        if record.levelno == logging.DEBUG:
            self._style._fmt = self.dbg_fmt

        elif record.levelno == logging.INFO:
            self._style._fmt = self.info_fmt

        elif record.levelno == logging.ERROR:
            self._style._fmt = self.err_fmt

        elif record.levelno == logging.WARNING:
            self._style._fmt = self.warn_fmt

        # Call the original formatter class to do the grunt work
        result = logging.Formatter.format(self, record)

        # Restore the original format configured by the user
        self._style._fmt = format_orig

        return result


def set_up_colorlog_logger(logger: logging.Logger):
    from colorlog import StreamHandler

    dbg_fmt = "%(log_color)s%(levelname)-8s %(cyan)s [%(filename)s:%(lineno)d] %(reset)s%(message)s"
    custom_formatter = CustomTmtccmdFormatter(
        info_fmt="%(log_color)s%(levelname)-8s %(cyan)s %(reset)s%(message)s",
        dbg_fmt=dbg_fmt,
        err_fmt=dbg_fmt,
        warn_fmt=dbg_fmt,
    )

    console_handler = StreamHandler(stream=sys.stdout)
    console_handler.setFormatter(custom_formatter)
    logger.addHandler(console_handler)
    logger.propagate = False
