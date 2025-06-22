import re
from pathlib import Path
from typing import List

from fsfwgen.parserbase.parser import FileParser
from fsfwgen.logging import get_console_logger
from fsfwgen.utility.sql_writer import SqlWriter


LOGGER = get_console_logger()


class ObjectDefinitionParser(FileParser):
    def __init__(self, file_list: List[Path]):
        super().__init__(file_list)

    def _handle_file_parsing(self, file_name: Path, *args, **kwargs):
        file = open(file_name, "r", encoding="utf-8")
        for line in file.readlines():
            match = re.search(r"([\w]*)[\s]*=[\s]*(0[xX][0-9a-fA-F]+)", line)
            if match:
                self.mib_table.update({match.group(2): [match.group(1)]})

    def _handle_file_parsing_moving_window(
        self,
        file_name: Path,
        current_line: int,
        moving_window_size: int,
        moving_window: list,
        *args,
        **kwargs,
    ):
        pass

    def _post_parsing_operation(self):
        pass


def export_object_file(filename, object_list, file_separator: str = ","):
    file = open(filename, "w")
    for entry in object_list:
        file.write(str(entry[0]) + file_separator + entry[1][0] + "\n")
    file.close()


def write_translation_file(filename: str, list_of_entries, date_string_full: str):
    with open(filename, "w") as out:
        LOGGER.info("ObjectParser: Writing translation file " + filename)
        definitions = ""
        function = (
            "const char *translateObject(object_id_t object) "
            "{\n  switch ((object & 0xFFFFFFFF)) {\n"
        )
        for entry in list_of_entries:
            # first part of translate file
            definitions += f'const char *{entry[1][0]}_STRING = "{entry[1][0]}";\n'
            # second part of translate file. entry[i] contains 32 bit hexadecimal numbers
            function += f"    case {entry[0]}:\n      return {entry[1][0]}_STRING;\n"
        function += '    default:\n      return "UNKNOWN_OBJECT";\n  }\n'
        out.write(
            f"/**\n * @brief  Auto-generated object translation file.\n"
            f" * @details\n"
            f" * Contains {len(list_of_entries)} translations.\n"
            f" * Generated on: {date_string_full}\n */\n"
        )
        out.write('#include "translateObjects.h"\n\n')
        out.write(definitions + "\n" + function + "  return 0;\n}\n")


def write_translation_header_file(filename: str = "translateObjects.h"):
    file = open(filename, "w")
    file.write(
        f"#ifndef FSFWCONFIG_OBJECTS_TRANSLATEOBJECTS_H_\n"
        f"#define FSFWCONFIG_OBJECTS_TRANSLATEOBJECTS_H_\n\n"
        f"#include <fsfw/objectmanager/SystemObjectIF.h>\n\n"
        f"const char *translateObject(object_id_t object);\n\n"
        f"#endif /* FSFWCONFIG_OBJECTS_TRANSLATEOBJECTS_H_ */\n"
    )


def sql_object_exporter(
    object_table: list,
    db_filename: str,
    delete_cmd: str,
    create_cmd: str,
    insert_cmd: str,
):
    sql_writer = SqlWriter(db_filename=db_filename)
    sql_writer.delete(delete_cmd)
    sql_writer.open(create_cmd)
    for entry in object_table:
        sql_writer.write_entries(insert_cmd, (entry[0], entry[1][0]))
    sql_writer.commit()
    sql_writer.close()
