import enum
import argparse


class ParserTypes(enum.Enum):
    EVENTS = "events"
    OBJECTS = "objects"
    RETVALS = "returnvalues"
    SUBSERVICES = "subservices"


def init_printout(project_string: str):
    print(f"-- {project_string} MIB Generator --")


def return_generic_args_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser("Arguments for FSFW MIB generation")
    choices = ("events", "objects", "returnvalues", "retvals", "subservices", "all")
    parser.add_argument(
        "type",
        metavar="type",
        choices=choices,
        help=f"Type of MIB data to generate. Choices: {choices}",
    )
    return parser
