# -*- coding: utf-8 -*-
import shutil
import os
from pathlib import Path

from fsfwgen.logging import get_console_logger

LOGGER = get_console_logger()


def copy_file(
    filename: Path, destination: Path = "", delete_existing_file: bool = False
):
    if not os.path.exists(filename):
        LOGGER.warning(f"File {filename} does not exist")
        return
    if not os.path.isdir(destination) and os.path.exists(destination):
        if delete_existing_file:
            os.remove(destination)
        else:
            LOGGER.warning(f"Destination file {destination} already exists")
            return
    try:
        shutil.copy2(src=filename, dst=destination)
    except FileNotFoundError:
        LOGGER.exception("File not found!")
    except shutil.SameFileError:
        LOGGER.exception("Source and destination are the same!")


def move_file(file_name: Path, destination: Path = ""):
    if not os.path.exists(file_name):
        print(f"move_file: File {file_name} does not exist")
        return
    if not os.path.exists(destination):
        print(f"move_file: Destination directory {destination} does not exist")
        return
    try:
        shutil.copy2(file_name, destination)
        os.remove(file_name)
        return
    except FileNotFoundError:
        LOGGER.exception("File not found!")
    except shutil.SameFileError:
        LOGGER.exception("Source and destination are the same!")
