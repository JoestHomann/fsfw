from pathlib import Path

from fsfwgen.utility.file_management import copy_file, move_file


# TODO: Export to SQL
class CsvWriter:
    def __init__(
        self,
        filename: Path,
        table_to_print=None,
        header_array=None,
        file_separator: str = ",",
    ):
        if header_array is None:
            header_array = []
        if table_to_print is None:
            table_to_print = dict()
        self.filename = filename
        self.table_to_print = table_to_print
        self.header_array = header_array
        if self.header_array != 0:
            self.column_numbers = len(self.header_array)
        self.file_separator = file_separator

    def write_to_csv(self):
        file = open(self.filename, "w")
        file.write("Index" + self.file_separator)
        for index in range(self.column_numbers):
            # noinspection PyTypeChecker
            if index < len(self.header_array) - 1:
                file.write(self.header_array[index] + self.file_separator)
            else:
                file.write(self.header_array[index] + "\n")
        for index, entry in self.table_to_print.items():
            file.write(str(index) + self.file_separator)
            for columnIndex in range(self.column_numbers):
                # noinspection PyTypeChecker
                if columnIndex < len(self.header_array) - 1:
                    file.write(str(entry[columnIndex]) + self.file_separator)
                else:
                    file.write(str(entry[columnIndex]) + "\n")
        file.close()

    def copy_csv(self, copy_destination: Path = "."):
        copy_file(self.filename, copy_destination)
        print(f"CSV file was copied to {copy_destination}")

    def move_csv(self, move_destination: Path):
        move_file(self.filename, move_destination)
        if move_destination == ".." or move_destination == "../":
            print("CSV Writer: CSV file was moved to parser root directory")
        else:
            print(f"CSV Writer: CSV file was moved to {move_destination}")
