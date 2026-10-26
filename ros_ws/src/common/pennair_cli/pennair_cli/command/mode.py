from argparse import ArgumentParser
from typing import override

from pennair_cli.extension import CommandExtension
from vehicle_common.mode_loader import ModeRegistry


# ModeRegistry causes a bunch of imports, which makes the cli pretty slow. Either optimize the import speed (opencv could be the culprit)
# or switch to reading static json file
class ModeCommand(CommandExtension):
    """Prints out general information about all registered modes"""

    @override
    def add_arguments(self, parser: ArgumentParser, cli_name: str):
        self.parser = parser

        subparsers = parser.add_subparsers(
            title="Commands",
            dest="mode_command",
            required=False,
        )

        ls_parser = subparsers.add_parser("ls", help="List registered modes.")
        ls_parser.set_defaults(func=self.ls)

        info_parser = subparsers.add_parser("info", help="Display info about specified modes")
        info_parser.set_defaults(func=self.show_info)

    @override
    def main(self, *, args):
        self.parser.print_help()

    def ls(self, *, args) -> None:
        for registered_mode in ModeRegistry.get().modes.values():
            print(registered_mode, end="\n\n")
        return

    def show_info(self, mode_id: str) -> None:
        registered_mode = ModeRegistry.get().modes.get(mode_id, None)
        if registered_mode is None:
            print(f"{id} is not present in the ModeRegistry")
            return
        print(registered_mode, end="\n\n")
