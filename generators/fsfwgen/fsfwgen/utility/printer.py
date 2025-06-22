import pprint

PrettyPrinter = pprint.PrettyPrinter(indent=0, width=250)


class Printer:
    def __init__(self):
        pass

    @staticmethod
    def print_content(dictionary, leading_string: str = ""):
        if leading_string != "":
            print(leading_string)
        PrettyPrinter.pprint(dictionary)
        print("\r\n", end="")
