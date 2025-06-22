"""Generic File Parser class
Used by parse header files. Implemented as class in case header parser becomes more complex
"""
from pathlib import Path
from typing import Union, List

from fsfwgen.logging import get_console_logger
from logging import DEBUG

LOGGER = get_console_logger()


# pylint: disable=too-few-public-methods
class FileListParser:
    """Generic header parser which takes a directory name or directory name list
    and parses all included header files recursively.
    TODO: Filter functionality for each directory to filter out files or folders
    """

    def __init__(self, directory_list_or_name: Union[Path, List[Path]]):
        self.directory_list = []
        if isinstance(directory_list_or_name, Path):
            self.directory_list.append(directory_list_or_name)
        elif isinstance(directory_list_or_name, List):
            self.directory_list.extend(directory_list_or_name)
        else:
            LOGGER.warning(
                "Header Parser: Passed directory list is not a header name or list of header names"
            )
        self.header_files = []

    def parse_header_files(
        self,
        search_recursively: bool = False,
        printout_string: str = "Parsing header files: ",
        print_current_dir: bool = False,
    ) -> List[Path]:
        """This function is called to get a list of header files
        :param search_recursively:
        :param printout_string:
        :param print_current_dir:
        :return:
        """
        print(printout_string, end="")
        for directory in self.directory_list:
            self.__get_header_file_list(
                directory, search_recursively, print_current_dir
            )
        print(str(len(self.header_files)) + " header files were found.")
        # g.PP.pprint(self.header_files)
        return self.header_files

    def __get_header_file_list(
        self,
        base_directory: Path,
        seach_recursively: bool = False,
        print_current_dir: bool = False,
    ):
        local_header_files = []
        if print_current_dir:
            print(f"Parsing header files in: {base_directory}")
        for entry in base_directory.iterdir():
            if (
                entry.is_file()
                and entry.suffix == ".h"
                and entry.as_posix()[0] not in [".", "_"]
            ):
                local_header_files.append(entry)
            if seach_recursively:
                if entry.is_dir():
                    self.__get_header_file_list(entry, seach_recursively)
        self.header_files.extend(local_header_files)
