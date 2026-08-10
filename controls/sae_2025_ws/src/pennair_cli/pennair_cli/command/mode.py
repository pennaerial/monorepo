from pennair_cli.extension import CommandExtension
from vehicle_common.mode_loader import ModeRegistry
from argparse import ArgumentParser


# ModeRegistry causes a bunch of imports, which makes the cli pretty slow. Either optimize the import speed (opencv could be the culprit)
# or switch to reading static json file
class ModeCommand(CommandExtension):
    """Prints out general information about all registered modes"""

    def add_arguments(self, parser: ArgumentParser, cli_name: str):
        self.parser = parser

        parser.add_argument(
            "--all",
            action="store_true",
            help="Print information about all registered modes.",
        )

        parser.add_argument(
            "mode_ids",
            nargs="*",
            help="Prints out information about all modes with the provided ids",
        )

    def main(self, *, args):
        print("mode command")
        self.registry = ModeRegistry.get()
        if not args.all and not args.mode_ids:
            self.parser.print_help()
            return

        if args.all:
            for registered_mode in self.registry.modes.values():
                print(registered_mode, end="\n\n")
            return

        for id in args.mode_ids:
            registered_mode = self.registry.modes.get(id, None)
            if registered_mode is None:
                print(f"{id} is not present in the ModeRegistry")
                continue
            print(registered_mode, end="\n\n")
