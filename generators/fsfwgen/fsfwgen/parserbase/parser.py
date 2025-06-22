#! /usr/bin/python3
"""
@file       packet_content_parser.py
@brief      Generic File Parser class
@details
Used by the MIB Exporter. There are multiple functions which are abstract and should
be implemented by a custom parser implementation.

A file list to parse must be supplied.
Child classes fill out the MIB table (self.mib_table)
@author     R. Mueller
@date       14.11.2019
"""
import enum
import re
from abc import abstractmethod
from pathlib import Path
from typing import Dict, List
from enum import Enum, auto


class VerbosityLevels(enum.Enum):
    REDUCED = 0
    REGULAR = 1
    DEBUG = 2


class FileParserModes(Enum):
    REGULAR = 1
    MOVING_WINDOW = 2


class FileParser:
    """
    This parent class gathers common file parser operations into a super class.
    The user should do the following to use this base class:
    1. Create a custom parser class which implements this class and implement the abstract
       functions
    2. Set the parser mode
    3. Call parse_files. Additional arguments and keyword arguments can be supplied as well and
       will be passed through to the abstract function implementations.
    """

    def __init__(self, file_list: List[Path]):
        if len(file_list) == 0:
            print("File list is empty !")
            self.file_list_empty = True
        else:
            self.file_list_empty = False
        self.file_list = file_list
        # Can be used to have unique key in MIB tables
        self.index = 0
        # Initialize empty MIB table which will be filled by specific parser implementation
        self.mib_table = dict()
        self.__parser_mode = FileParserModes.REGULAR
        self.__parser_args = 0

        self.__debug_moving_window = False
        self.__debug_moving_window_filename = ""
        self._moving_window_center_idx = 3

        self._verbose_level = 1

    def set_regular_parser_mode(self):
        """
        Set regular parsing mode. This will be the default, so it is not strictly necessary to call
        this.
        :return:
        """
        self.__parser_mode = FileParserModes.REGULAR

    def set_moving_window_mode(self, moving_window_size: int):
        """
        Set moving window parsing mode
        :param moving_window_size:
        :return:
        """
        self.__parser_mode = FileParserModes.MOVING_WINDOW
        self.__parser_args = moving_window_size

    def set_verbosity(self, verbose_level: int):
        self._verbose_level = verbose_level

    def get_verbosity(self):
        return self._verbose_level

    def enable_moving_window_debugging(self, file_name: str):
        self.__debug_moving_window = True
        self.__debug_moving_window_filename = file_name

    def parse_files(self, *args: any, **kwargs) -> Dict:
        """
        Core method which is called to parse the files
        :param args: Optional positional arguments. Passed on the file parser
        :param kwargs: Optional keyword arguments. Passed on to file parser
        :return: Returns the mib table dictionary.
        """
        if self.file_list_empty:
            print(f"Nothing to parse, supplied file list is empty!")
            return self.mib_table

        if self.__parser_mode == FileParserModes.REGULAR:
            for file_name in self.file_list:
                # Implemented by child class ! Fill out info table (self.mib_table) in this routine
                self._handle_file_parsing(file_name, *args, **kwargs)
            # Can be implemented by child class to edit the table after it is finished.
            # default implementation is empty
            self._post_parsing_operation()
        elif self.__parser_mode == FileParserModes.MOVING_WINDOW:
            for file_name in self.file_list:
                self.__parse_file_with_moving_window(file_name, *args, **kwargs)
            self._post_parsing_operation()
        return self.mib_table

    @abstractmethod
    def _handle_file_parsing(self, file_name: Path, *args, **kwargs):
        """
        Implemented by child class. The developer should fill the info table (self.mib_table)
        in this routine
        :param file_name:
        :param args:    Additional arguments passed through the parse_files method.
        :param kwargs:  Additional keyword arguments passed through the parse_files method.
        :return: Nothing. Fill out the member dictionary self.mib_table in the function instead.
        """
        pass

    @abstractmethod
    def _handle_file_parsing_moving_window(
        self,
        file_name: Path,
        current_line: int,
        moving_window_size: int,
        moving_window: list,
        *args,
        **kwargs,
    ):
        """
        This will be called for the MOVING_WINDOW parser mode.
        :param file_name:           Current file name
        :param current_line:        Current line number.
        :param moving_window_size:  Size of the moving window
        :param moving_window:       Current moving window. The last entry of the moving window
                                    is the current line number
        :return: Nothing. Fill out the member dictionary self.mib_table in the function instead.
        """
        pass

    @abstractmethod
    def _post_parsing_operation(self):
        """
        # Can be implemented by child class to perform post parsing operations (e.g. setting a
        flag or editting MIB table entries)
        :return:
        """

    def __parse_file_with_moving_window(self, file_name: Path, *args, **kwargs):
        all_lines = self._open_file(file_name=file_name)
        moving_window_size = self.__parser_args
        if moving_window_size == 0:
            print("Moving window size is 0!")
            return
        moving_window = [""] * moving_window_size
        for line_idx, line in enumerate(all_lines):
            if (
                self.__debug_moving_window
                and self.__debug_moving_window_filename in file_name
            ):
                print(f"Moving window pre line anaylsis line {line_idx}")
                print(moving_window)
            # The moving window will start with only the bottom being in the file
            if line_idx == 0:
                moving_window[self.__parser_args - 1] = line
            # More and more of the window is inside the file now
            elif line_idx < moving_window_size:
                for idx in range(line_idx, 0, -1):
                    moving_window[moving_window_size - 1 - idx] = moving_window[
                        moving_window_size - idx
                    ]
                moving_window[moving_window_size - 1] = line
            # The full window is inside the file now.
            elif line_idx >= moving_window_size:
                for idx in range(moving_window_size - 1):
                    moving_window[idx] = moving_window[idx + 1]
                moving_window[moving_window_size - 1] = line
            if (
                self.__debug_moving_window
                and self.__debug_moving_window_filename in file_name
            ):
                print(f"Moving window post line analysis line {line_idx}")
                print(moving_window)
            self._handle_file_parsing_moving_window(
                file_name, line_idx, moving_window_size, moving_window, *args, **kwargs
            )
        # Now the moving window moved past the end of the file. Sections which are outside
        # the file are assigned an empty string until the window has moved out of file completely
        for remaining_windows_idx in range(moving_window_size):
            if (
                self.__debug_moving_window
                and self.__debug_moving_window_filename in file_name
            ):
                print(f"Moving window pre line analysis post EOF")
                print(moving_window)
            num_entries_to_clear = remaining_windows_idx + 1
            for idx_to_clear in range(num_entries_to_clear):
                moving_window[moving_window_size - 1 - idx_to_clear] = ""
            for idx_to_reassign in range(moving_window_size - 1 - num_entries_to_clear):
                moving_window[idx_to_reassign] = moving_window[idx_to_reassign + 1]
            if (
                self.__debug_moving_window
                and self.__debug_moving_window_filename in file_name
            ):
                print(f"Moving window post line anaylsis post EOF")
                print(moving_window)
        pass

    @staticmethod
    def _open_file(file_name: Path) -> list:
        """
        Open a file, attempting common encodings utf-8 and cp1252
        :param file_name:
        :return:
        """
        try:
            file = open(file_name, "r", encoding="utf-8")
            all_lines = file.readlines()
        except UnicodeDecodeError:
            print(f"Parser: Decoding error with file {file_name}")
            file = open(file_name, "r", encoding="cp1252")
            all_lines = file.readlines()
        return all_lines

    def _build_multi_line_string_generic(
        self, first_line: str, moving_window: List[str]
    ) -> str:
        """This function transforms a multi line match into a one line match by searching for the
        semicolon at the string end"""
        all_lines = first_line.rstrip()
        end_found = False
        current_idx = self._moving_window_center_idx
        while not end_found and current_idx < len(moving_window) - 1:
            current_idx += 1
            string_to_add = moving_window[current_idx].lstrip()
            if ";" in moving_window[current_idx]:
                all_lines += string_to_add
                break
            else:
                string_to_add.rstrip()
                all_lines += string_to_add
        return all_lines

    def _search_for_descrip_string_generic(
        self, moving_window: List[str], break_pattern: str
    ) -> str:
        current_idx = self._moving_window_center_idx - 1
        # Look at the line above first
        descrip_match = re.search(
            r"\[EXPORT][\s]*:[\s]*\[COMMENT]", moving_window[current_idx]
        )
        if not descrip_match:
            while True:
                if re.search(break_pattern, moving_window[current_idx]):
                    break
                descrip_match = re.search(
                    r"\[EXPORT][\s]*:[\s]*\[COMMENT]", moving_window[current_idx]
                )
                if descrip_match or current_idx <= 0:
                    break
                current_idx -= 1
        if descrip_match:
            current_build_idx = current_idx
            descrip_string = ""
            while current_build_idx < self._moving_window_center_idx:
                string_to_add = moving_window[current_build_idx].lstrip()
                string_to_add = string_to_add.lstrip("//!<>")
                string_to_add = string_to_add.rstrip()
                descrip_string += string_to_add
                current_build_idx += 1
        else:
            return ""
        resulting_description = re.search(
            r"\[EXPORT][\s]*:[\s]*\[COMMENT][\s](.*)", descrip_string
        )
        if resulting_description:
            return resulting_description.group(1)
        return ""
