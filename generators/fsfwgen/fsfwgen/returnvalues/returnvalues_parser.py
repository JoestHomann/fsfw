import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Union

from fsfwgen.parserbase.parser import FileParser, VerbosityLevels
from fsfwgen.utility.printer import PrettyPrinter
from fsfwgen.logging import get_console_logger

LOGGER = get_console_logger()

# Intermediate solution
MAX_STRING_LEN = 80

PRINT_TRUNCATED_ENTRIES = False
DEBUG_FOR_FILE_NAME = False

CLASS_ID_NAMESPACE = "CLASS_ID"
DEFAULT_MOVING_WINDOWS_SIZE = 7
INVALID_IF_ID = -1


@dataclass
class FileStartHelper:
    start_name: str
    start_name_or_value: str
    count: int
    cumulative_start_index: Optional[int] = None


@dataclass
class FileEndHelper:
    end_name: str
    cumulative_end_value: Optional[int] = None


@dataclass
class FileConnHelper:
    file_name: str
    sh: Optional[FileStartHelper]
    eh: Optional[FileEndHelper]


class InterfaceParser(FileParser):
    def __init__(self, file_list: list, print_table: bool = False):
        super().__init__(file_list)
        self.print_table = print_table
        self.file_table_list = []
        self.file_name_table = []
        self.file_conn_helpers: Optional[List[FileConnHelper]] = None
        self._debug_mode = False

    def enable_debug_mode(self, enable: bool):
        self._debug_mode = enable

    def _handle_file_parsing_moving_window(
        self,
        file_name: str,
        current_line: int,
        moving_window_size: int,
        moving_window: list,
        *args,
        **kwargs,
    ):
        pass

    def _handle_file_parsing(self, file_name: str, *args, **kwargs):
        self.file_name_table.append(file_name)
        try:
            file = open(file_name, "r", encoding="utf-8")
            all_lines = file.readlines()
        except UnicodeDecodeError:
            file = open(file_name, "r", encoding="cp1252")
            all_lines = file.readlines()
        self.__handle_regular_class_id_parsing(file_name=file_name, all_lines=all_lines)

    def __handle_regular_class_id_parsing(self, file_name: str, all_lines: List[str]):
        count = 0
        current_file_table = dict()
        start_matched = False
        end_matched = False
        start_name = ""
        first_entry_name_or_index = ""
        file_conn_entry = FileConnHelper(file_name, None, None)
        for line in all_lines:
            if not start_matched:
                match = re.search(r"[\s]*([\w]*) = [\s]*([\w]*)", line)
                if match:
                    # current_file_table.update({count: [match.group(1), match.group(2)]})
                    start_name = match.group(1)
                    first_entry_name_or_index = match.group(2)
                    start_matched = True
            else:
                match = re.search(r"[\s]*([\w]*),?(?:[\s]*//)?([^\n]*)?", line)
                if match:
                    count += 1
                    # It is expected that the last entry is explicitely marked like this.
                    # TODO: Could also simply remember last entry and then designate that as end
                    #       entry as soon as "}" is found. Requires moving window mode though
                    if re.search(r"\[EXPORT][\s]*:[\s]*\[END]", match.group(2)):
                        last_entry_name = match.group(1)
                        end_matched = True
                        file_conn_entry.eh = FileEndHelper(last_entry_name, None)
                        break
                    else:
                        short_name = match.group(2)
                        if short_name == "":
                            short_name = match.group(1)[0:3]
                        current_file_table.update({count: [match.group(1), short_name]})
        if not start_matched:
            print("No start match detected when parsing interface files..")
            print(
                f"Current file: {file_name} | Make sure to include a start definition"
            )
            sys.exit(1)
        if not end_matched:
            print(
                "No end match detected when parsing interface files. "
                "Make sure to use [EXPORT] : [END]"
            )
            sys.exit(1)
        file_conn_entry.sh = FileStartHelper(
            start_name, first_entry_name_or_index, count, None
        )
        if self.file_conn_helpers is None:
            self.file_conn_helpers = []
        self.file_conn_helpers.append(file_conn_entry)
        self.file_name_table.append(file_name)
        self.file_table_list.append(current_file_table)

    def _post_parsing_operation(self):
        self.__assign_start_end_indexes()
        self._print_start_end_info()
        for idx, file_table in enumerate(self.file_table_list):
            self.__build_mod_interface_table(
                self.file_conn_helpers[idx].sh.cumulative_start_index, file_table
            )
        if self.print_table:
            PrettyPrinter.pprint(self.mib_table)

    def _print_start_end_info(self):
        for conn_helper in self.file_conn_helpers:
            print(
                f"Detected {conn_helper.sh.count} entries in {conn_helper.file_name}, "
                f"end index {conn_helper.eh.cumulative_end_value}"
            )

    def __assign_start_end_indexes(self):
        conn_helpers_old = self.file_conn_helpers.copy()
        all_indexes_filled = False
        max_outer_iterations = 15
        current_iteration = 0
        while not all_indexes_filled:
            for idx, conn_helper in enumerate(conn_helpers_old):
                sh = conn_helper.sh
                # In the very first file, the first index might/will be a number
                if sh.start_name_or_value.isdigit():
                    sh.cumulative_start_index = int(sh.start_name_or_value)
                    conn_helpers_old[idx].eh.cumulative_end_value = (
                        sh.cumulative_start_index + sh.count
                    )
                # Now, we try to connect the start and end of the files using the start and end
                # names respectively
                end_name_to_search = conn_helper.sh.start_name_or_value
                for end_name_helper in conn_helpers_old:
                    eh = end_name_helper.eh
                    if (
                        eh.end_name == end_name_to_search
                        and eh.cumulative_end_value is not None
                    ):
                        self.file_conn_helpers[
                            idx
                        ].sh.cumulative_start_index = eh.cumulative_end_value
                        self.file_conn_helpers[idx].eh.cumulative_end_value = (
                            eh.cumulative_end_value
                            + self.file_conn_helpers[idx].sh.count
                        )
            all_indexes_filled = True
            for idx, conn_helper in enumerate(conn_helpers_old):
                if (
                    conn_helper.sh.cumulative_start_index is None
                    or conn_helper.eh.cumulative_end_value is None
                ):
                    all_indexes_filled = False
            current_iteration += 1
            if current_iteration >= max_outer_iterations:
                print(
                    "Could not fill out start and end index list in "
                    "given number of maximum outer iterations!"
                )
                sys.exit(1)

    def __build_mod_interface_table(self, count_start: int, interface_dict: dict):
        dict_to_build = dict()
        for local_count, interface_name_and_shortname in interface_dict.items():
            dict_to_build.update(
                {
                    interface_name_and_shortname[0]: [
                        local_count + count_start,
                        interface_name_and_shortname[1],
                    ]
                }
            )
        self.mib_table.update(dict_to_build)


class RetvalEntry:
    def __init__(
        self,
        name: str,
        description: str,
        unique_id: int,
        file_name: Path,
        subsystem_name: str,
    ):
        self.name = name
        self.description = description
        self.unique_id = unique_id
        self.file_name = file_name
        self.subsystem_name = subsystem_name

    def __repr__(self):
        return (
            f"RetvalEntry(name={self.name!r}, description={self.description!r}, "
            f"unique_id={self.unique_id!r}, file_name={self.file_name!r}, "
            f"subsystem_name={self.subsystem_name!r})"
        )


RetvalDictT = Dict[int, RetvalEntry]


class ReturnValueParser(FileParser):
    """
    Generic return value parser.
    """

    def __init__(self, interfaces, file_list, print_tables):
        super().__init__(file_list)
        self.print_tables = print_tables
        self.interfaces = interfaces
        self.return_value_dict = dict()
        self.count = 0
        # Stores last three lines
        self.last_lines = ["", "", ""]
        self.obsw_root_path: Optional[Path] = None
        self.current_interface_id_entries = {"Name": "", "ID": 0, "FullName": ""}
        self.return_value_dict.update(
            {
                0: RetvalEntry(
                    name="OK",
                    description="System-wide code for ok.",
                    unique_id=0,
                    file_name=Path("fsfw/returnvalues/returnvalue.h"),
                    subsystem_name="HasReturnvaluesIF",
                )
            }
        )
        self.return_value_dict.update(
            {
                1: RetvalEntry(
                    name="Failed",
                    description="Unspecified system-wide code for failed.",
                    unique_id=1,
                    file_name=Path("fsfw/returnvalues/returnvalue.h"),
                    subsystem_name="HasReturnvaluesIF",
                )
            }
        )

    def _handle_file_parsing(self, file_name: Path, *args, **kwargs):
        """Former way to parse returnvalues. Not recommended anymore.
        :param file_name:
        :param args:
        :param kwargs:
        :return:
        """
        if len(args) > 0:
            print_truncated_entries = args[0]
        else:
            print_truncated_entries = False
        all_lines = self._open_file(file_name=file_name)
        for line in all_lines:
            self.__handle_line_reading(line, file_name, print_truncated_entries)

    def _handle_file_parsing_moving_window(
        self,
        file_name: Path,
        current_line: int,
        moving_window_size: int,
        moving_window: list,
        *args,
        **kwargs,
    ):
        """Parse for returnvalues using a moving window"""
        interface_id_match = re.search(
            rf"{CLASS_ID_NAMESPACE}::([a-zA-Z_0-9]*)",
            moving_window[self._moving_window_center_idx],
        )

        if interface_id_match:
            self.__handle_interfaceid_match(
                interface_id_match=interface_id_match, file_name=file_name
            )
        returnvalue_match = re.search(
            r"^[\s]*static const(?:expr)?[\s]*ReturnValue_t[\s]*([\w]*)[\s]*=[\s]*((?!;).*$)",
            moving_window[self._moving_window_center_idx],
            re.DOTALL,
        )
        full_returnvalue_string = ""
        if returnvalue_match:
            if ";" in returnvalue_match.group(0):
                full_returnvalue_string = returnvalue_match.group(0)
            else:
                full_returnvalue_string = self.__build_multi_line_returnvalue_string(
                    moving_window=moving_window,
                    first_line=moving_window[self._moving_window_center_idx],
                )
        number_match = INVALID_IF_ID
        # Try to match for a string using the new API first. Example:
        # static const ReturnValue_t PACKET_TOO_LONG =
        #       returnvalue::makeCode(CLASS_ID, 0);
        returnvalue_match = re.search(
            r"^[\s]*static const(?:expr)? ReturnValue_t[\s]*([\w]*)[\s]*"
            r"=[\s]*.*::[\w]*\(([\w]*),[\s]*([\d]*)\)",
            full_returnvalue_string,
        )
        if not returnvalue_match:
            # Try to match for old API using MAE_RETURN_CODE macro
            returnvalue_match = re.search(
                r"^[\s]*static const(?:expr)? ReturnValue_t[\s]*([a-zA-Z_0-9]*)[\s]*=[\s]*"
                r"MAKE_RETURN_CODE[\s]*\([\s]*([\w]*)[\s]*\)",
                full_returnvalue_string,
            )
            if returnvalue_match:
                number_match = get_number_from_dec_or_hex_str(
                    returnvalue_match.group(2)
                )
        else:
            number_match = get_number_from_dec_or_hex_str(returnvalue_match.group(3))
        if returnvalue_match:
            description = self.__search_for_descrip_string(moving_window=moving_window)
            if number_match == INVALID_IF_ID:
                LOGGER.warning(f"Invalid number match detected for file {file_name}")
                LOGGER.warning(f"Match groups:")
                for group in returnvalue_match.groups():
                    LOGGER.info(group)
            self.__handle_returnvalue_match(
                name_match=returnvalue_match.group(1),
                file_name=file_name,
                number_match=number_match,
                description=description,
            )

    def __build_multi_line_returnvalue_string(
        self, first_line: str, moving_window: List[str]
    ) -> str:
        return self._build_multi_line_string_generic(
            first_line=first_line, moving_window=moving_window
        )

    def __search_for_descrip_string(self, moving_window: List[str]) -> str:
        return self._search_for_descrip_string_generic(
            moving_window=moving_window,
            break_pattern=r"^[\s]*static const(?:expr)? ReturnValue_t",
        )

    def __handle_line_reading(self, line, file_name, print_truncated_entries: bool):
        newline = line
        if self.last_lines[0] != "\n":
            two_lines = self.last_lines[0] + " " + newline.strip()
        else:
            two_lines = ""
        interface_id_match = re.search(
            r"INTERFACE_ID[\s]*=[\s]*CLASS_ID::([a-zA-Z_0-9]*)", two_lines
        )
        if interface_id_match:
            self.__handle_interfaceid_match(interface_id_match, file_name=file_name)

        returnvalue_match = re.search(
            r"^[\s]*static const(?:expr)? ReturnValue_t[\s]*([a-zA-Z_0-9]*)[\s]*=[\s]*"
            r"MAKE_RETURN_CODE[\s]*\([\s]*([x0-9a-fA-F]{1,4})[\s]*\);[\t ]*(//)?([^\n]*)",
            two_lines,
        )
        if returnvalue_match:
            self.__handle_returnvalue_match(
                name_match=returnvalue_match.group(1),
                file_name=file_name,
                description="",
                number_match=get_number_from_dec_or_hex_str(returnvalue_match.group(2)),
            )
        self.last_lines[1] = self.last_lines[0]
        self.last_lines[0] = newline

    def __handle_interfaceid_match(self, interface_id_match, file_name: Path) -> bool:
        """Handle a match of an interface ID definition in the code.
        Returns whether the interface ID was found successfully in the IF ID header files
        """
        if self.get_verbosity() == VerbosityLevels.DEBUG:
            LOGGER.info(
                f"Interface ID {interface_id_match.group(1)} found in {file_name}"
            )
        if_id_entry = self.interfaces.get(interface_id_match.group(1))
        if if_id_entry is not None:
            self.current_interface_id_entries["ID"] = if_id_entry[0]
        else:
            LOGGER.warning(
                f"Interface ID {interface_id_match.group(1)} not found in IF ID dictionary"
            )
            return False
        self.current_interface_id_entries["Name"] = self.interfaces[
            interface_id_match.group(1)
        ][1].lstrip()
        self.current_interface_id_entries["FullName"] = interface_id_match.group(1)
        if self.get_verbosity() == VerbosityLevels.DEBUG:
            current_id = self.current_interface_id_entries["ID"]
            LOGGER.info(f"Current ID: {current_id}")
        return True

    def __handle_returnvalue_match(
        self, name_match: str, number_match: int, file_name: Path, description: str
    ):
        string_to_add = self.build_checked_string(
            self.current_interface_id_entries["Name"],
            name_match,
            MAX_STRING_LEN,
            PRINT_TRUNCATED_ENTRIES,
        )
        full_id = (self.current_interface_id_entries["ID"] << 8) | number_match
        if full_id in self.return_value_dict:
            # print('Duplicate returncode ' + hex(full_id) + ' from ' + file_name +
            #      ' was already in ' + self.return_value_dict[full_id][3])
            pass
        if self.obsw_root_path is not None:
            file_name = file_name.relative_to(self.obsw_root_path)
        mib_entry = RetvalEntry(
            name=string_to_add,
            description=description,
            unique_id=number_match,
            file_name=file_name,
            subsystem_name=self.current_interface_id_entries["FullName"],
        )
        self.return_value_dict.update({full_id: mib_entry})
        self.count = self.count + 1

    def _post_parsing_operation(self):
        if self.print_tables:
            PrettyPrinter.pprint(self.return_value_dict)
        self.mib_table = self.return_value_dict

    @staticmethod
    def export_to_csv(filename: Path, list_of_entries: RetvalDictT, column_sep: str):
        with open(filename, "w") as out:
            out.write(
                f"Full ID (hex){column_sep} Name{column_sep} Description{column_sep} "
                f"Unique ID{column_sep} Subsytem Name{column_sep} File Path\n"
            )
            for entry in list_of_entries.items():
                if column_sep == ";":
                    entry[1].description = entry[1].description.replace(";", ",")
                elif column_sep == ",":
                    # Quote the description
                    entry[1].description = f'"{entry[1].description}"'
                out.write(
                    f"{entry[0]:#06x}{column_sep}{entry[1].name}{column_sep}{entry[1].description}"
                    f"{column_sep}{entry[1].unique_id}{column_sep}{entry[1].subsystem_name}"
                    f"{column_sep}{entry[1].file_name.as_posix()}\n"
                )

    def build_checked_string(
        self,
        first_part,
        second_part,
        max_string_len: int,
        print_truncated_entries: bool,
    ):
        """Build a checked string"""
        my_str = first_part + "_" + self.convert(second_part)
        if len(my_str) > max_string_len:
            if print_truncated_entries:
                LOGGER.warning(f"Entry {my_str} too long. Will truncate.")
            my_str = my_str[0:max_string_len]
        else:
            # print("Entry: " + myStr + " is all right.")
            pass
        return my_str

    @staticmethod
    def convert(name):
        single_strings = name.split("_")
        new_string = ""
        for one_string in single_strings:
            one_string = one_string.lower()
            one_string = one_string.capitalize()
            new_string = new_string + one_string
        return new_string

    @staticmethod
    def clean_up_description(descr_string):
        description = descr_string.lstrip("!<- ")
        if description == "":
            description = " "
        return description


def get_number_from_dec_or_hex_str(a_string):
    if a_string.startswith("0x"):
        return int(a_string, 16)
    if a_string.isdigit():
        return int(a_string)
    LOGGER.warning(f"Illegal number representation: {a_string}")
    return 0
