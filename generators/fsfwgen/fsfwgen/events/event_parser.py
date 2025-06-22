import re
import logging
from pathlib import Path
from typing import List, Optional, Dict

from fsfwgen.parserbase.parser import FileParser
from fsfwgen.logging import get_console_logger

LOGGER = get_console_logger()


FSFW_EVENT_HEADER_INCLUDE = '#include "fsfw/events/Event.h"'
DEFAULT_MOVING_WINDOWS_SIZE = 7
SUBSYSTEM_ID_NAMESPACE = "SUBSYSTEM_ID"

EVENT_NAME_IDX = 1


class SubsystemDefinitionParser(FileParser):
    def __init__(self, file_list):
        super().__init__(file_list)
        self.moving_window_center_idx = 3

    def _handle_file_parsing(self, file_name: str, *args, **kwargs):
        file = open(file_name, "r")
        for line in file.readlines():
            match = re.search(r"([A-Z0-9_]*) = ([0-9]{1,3})", line)
            if match:
                self.mib_table.update({match.group(1): [match.group(2)]})

    def _handle_file_parsing_moving_window(
        self,
        file_name: str,
        current_line: int,
        moving_window_size: int,
        moving_window: list,
        *args,
        **kwargs,
    ):
        match = re.search(
            r"([A-Z0-9_]*) = ([0-9]{1,3})", moving_window[self.moving_window_center_idx]
        )
        if match:
            self.mib_table.update({match.group(1): [match.group(2)]})

    def _post_parsing_operation(self):
        pass


class EventEntry:
    def __init__(self, name: str, severity: str, description: str, file_name: Path):
        self.name = name
        self.severity = severity
        self.description = description
        self.file_name = file_name

    def __repr__(self):
        return (
            f"EventEntry(name={self.name!r}, severity={self.severity!r}, "
            f"description={self.description!r}, file_name={self.file_name!r}"
        )


EventDictT = Dict[int, EventEntry]


class EventParser(FileParser):
    def __init__(
        self, file_list: List[Path], interface_list, moving_window_size: int = 7
    ):
        super().__init__(file_list)
        self.set_moving_window_mode(moving_window_size)
        self.interfaces = interface_list
        self.count = 0
        self.my_id = 0
        self.current_id = 0
        self.mib_table: EventDictT = dict()
        self.obsw_root_path: Optional[Path] = None
        self.last_lines = ["", "", ""]
        self.moving_window_center_idx = 3

    def _handle_file_parsing(self, file_name: Path, *args: any, **kwargs):
        logging.warning("Regular file parsing mode not implemented")

    def _handle_file_parsing_moving_window(
        self,
        file_name: Path,
        current_line: int,
        moving_window_size: int,
        moving_window: list,
        *args,
        **kwargs,
    ):
        subsystem_id_assignment_match = re.search(
            rf"([\w]*)[\s]*=[\s]*{SUBSYSTEM_ID_NAMESPACE}::([A-Z_0-9]*);",
            moving_window[self.moving_window_center_idx],
        )
        if subsystem_id_assignment_match:
            # For now, it is assumed that there is only going to be one subsystem ID per
            # class / source file
            try:
                self.current_id = self.interfaces[
                    subsystem_id_assignment_match.group(2)
                ][0]
                self.my_id = self.return_number_from_string(self.current_id)
            except KeyError as e:
                print(f"Key not found: {e}")
        # Now try to look for event definitions. Moving windows allows multi line event definitions
        # These two variants need to be checked
        event_match = re.match(
            r"[\s]*static const(?:expr)?[\s]*Event[\s]*([\w]*)[\s]*=([^\n]*)",
            moving_window[self.moving_window_center_idx],
        )
        macro_api_match = False
        if event_match is not None:
            valid_event = False
            for idx in range(3):
                if "MAKE_EVENT" in moving_window[self.moving_window_center_idx + idx]:
                    macro_api_match = True
                    valid_event = True
                    break
                elif "makeEvent" in moving_window[self.moving_window_center_idx + idx]:
                    valid_event = True
                    break
            if not valid_event:
                event_match = False

        if event_match:
            self.__handle_event_match(
                event_match=event_match,
                macro_api_match=macro_api_match,
                moving_window=moving_window,
                file_name=file_name,
            )

    def __handle_event_match(
        self, event_match, macro_api_match: bool, moving_window: list, file_name: Path
    ):
        if ";" in event_match.group(0):
            event_full_match = self.__generate_regex_event_match(
                macro_api_match=macro_api_match,
                full_string=moving_window[self.moving_window_center_idx],
            )
        else:
            multi_line_string = self.__build_multi_line_event_string(
                first_line=event_match.group(0), moving_window=moving_window
            )
            event_full_match = self.__generate_regex_event_match(
                macro_api_match=macro_api_match, full_string=multi_line_string
            )
        description = self._search_for_descrip_string_generic(
            moving_window=moving_window,
            break_pattern=r"[\s]*static const(?:expr)?[\s]*Event[\s]*",
        )
        if event_full_match:
            name = event_match.group(EVENT_NAME_IDX)
            if macro_api_match:
                full_id = (self.my_id * 100) + self.return_number_from_string(
                    event_full_match.group(2)
                )
                severity = event_full_match.group(3)
            else:
                full_id = (self.my_id * 100) + self.return_number_from_string(
                    event_full_match.group(3)
                )
                severity = event_full_match.group(4)
            if self.obsw_root_path is not None:
                file_name = file_name.relative_to(self.obsw_root_path)
            if self.mib_table.get(full_id) is not None:
                LOGGER.warning(f"Duplicate event ID {full_id} detected")
                LOGGER.info(
                    f"Name: {self.mib_table.get(full_id).name}| "
                    f"Description: {self.mib_table.get(full_id).description}"
                )
            self.mib_table.update(
                {full_id: EventEntry(name, severity, description, file_name)}
            )
            self.count = self.count + 1

    @staticmethod
    def __generate_regex_event_match(macro_api_match: bool, full_string: str):
        if macro_api_match:
            # One line event definition.
            regex_string = (
                r"static const(?:expr)? Event[\s]*([\w]*)[\s]*=[\s]*"
                r"MAKE_EVENT\(([0-9]{1,3}),[\s]*severity::([A-Z]*)\)[\s]*;"
            )
        else:
            regex_string = (
                r"static const(?:expr)? Event[\s]*([\w]*)[\s]*=[\s]*"
                r"event::makeEvent\(([\w]*),[\s]*([0-9]{1,3})[\s]*,[\s]*severity::([A-Z]*)\)[\s]*;"
            )
        event_full_match = re.search(regex_string, full_string)
        return event_full_match

    def __build_multi_line_event_string(
        self, first_line: str, moving_window: List[str]
    ) -> str:
        return self._build_multi_line_string_generic(
            first_line=first_line, moving_window=moving_window
        )

    def _post_parsing_operation(self):
        pass

    def build_checked_string(self, first_part, second_part):
        my_str = first_part + self.convert(second_part)
        if len(my_str) > 16:
            print(f"EventParser: Entry: {my_str} too long. Will truncate.")
            my_str = my_str[0:14]
        return my_str

    @staticmethod
    def return_number_from_string(a_string):
        if a_string.startswith("0x"):
            return int(a_string, 16)
        elif a_string.isdigit():
            return int(a_string)
        else:
            print("EventParser: Illegal number representation: " + a_string)
            return 0

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
    def clean_up_description(description):
        description = description.lstrip("//!<>")
        description = description.lstrip()
        if description == "":
            description = " "
        return description


def export_to_csv(filename: Path, event_list: EventDictT, col_sep: str):
    with open(filename, "w") as out:
        fsep = col_sep
        out.write(
            f"Event ID (dec){col_sep} Event ID (hex){col_sep} Name{col_sep} "
            f"Severity{col_sep} Description{col_sep} File Path\n"
        )
        for entry in event_list.items():
            event_id = int(entry[0])
            event_value = entry[1]
            event_id_as_hex = f"{event_id:#06x}"
            out.write(
                f"{event_id}{fsep}{event_id_as_hex}{fsep}{event_value.name}{fsep}"
                f"{event_value.severity}{fsep}{event_value.description}"
                f"{fsep}{event_value.file_name.as_posix()}\n"
            )


def write_translation_source_file(
    event_list: EventDictT, date_string: str, filename: Path = "translateEvents.cpp"
):
    with open(filename, "w") as out:
        definitions = ""
        # Look up table to avoid duplicate events
        lut = dict()
        function = "const char *translateEvents(Event event) {\n  switch ((event & 0xFFFF)) {\n"
        for entry in event_list.items():
            event_id = entry[0]
            event_value = entry[1]
            name = event_value.name
            if name not in lut:
                definitions += f"const char *{name}_STRING " f'= "{name}";\n'
                function += f"    case ({event_id}):\n      " f"return {name}_STRING;\n"
                lut.update({name: event_value})
        function += '    default:\n      return "UNKNOWN_EVENT";\n'
        out.write(
            f"/**\n * @brief    Auto-generated event translation file. "
            f"Contains {len(event_list)} translations.\n"
            f" * @details\n"
            f" * Generated on: {date_string}\n */\n"
        )
        out.write('#include "translateEvents.h"\n\n')
        out.write(definitions + "\n" + function + "  }\n  return 0;\n}\n")


def write_translation_header_file(filename: Path = "translateEvents.h"):
    with open(filename, "w") as out:
        out.write(
            f"#ifndef FSFWCONFIG_EVENTS_TRANSLATEEVENTS_H_\n"
            f"#define FSFWCONFIG_EVENTS_TRANSLATEEVENTS_H_\n\n"
            f"{FSFW_EVENT_HEADER_INCLUDE}\n\n"
            f"const char *translateEvents(Event event);\n\n"
            f"#endif /* FSFWCONFIG_EVENTS_TRANSLATEEVENTS_H_ */\n"
        )


def handle_csv_export(file_name: Path, event_list: EventDictT, file_separator: str):
    """
    Generates the CSV in the same directory as the .py file and copes the CSV to another
    directory if specified.
    """
    export_to_csv(filename=file_name, event_list=event_list, col_sep=file_separator)


def handle_cpp_export(
    event_list: EventDictT,
    date_string: str,
    file_name: Path = "translateEvents.cpp",
    generate_header: bool = True,
    header_file_name: Path = "translateEvents.h",
):
    write_translation_source_file(
        event_list=event_list, date_string=date_string, filename=file_name
    )
    if generate_header:
        write_translation_header_file(filename=header_file_name)
