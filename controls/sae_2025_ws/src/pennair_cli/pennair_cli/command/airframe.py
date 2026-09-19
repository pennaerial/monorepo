from argparse import ArgumentParser
from typing import override

from pennair_cli.extension import CommandExtension
from uav.vehicles.AirframeClass import PX4Airframe


class AirframeCommand(CommandExtension):
    """Prints out available UAV airframes."""

    @override
    def add_arguments(self, parser: ArgumentParser, cli_name: str):
        self.parser = parser

        subparsers = parser.add_subparsers(
            title="Commands",
            dest="airframe_command",
            required=False,
        )

        ls_parser = subparsers.add_parser("ls", help="List available airframes.")
        ls_parser.set_defaults(func=self.ls)

    @override
    def main(self, *, args):
        self.parser.print_help()

    def ls(self, *, args) -> None:
        print("Available airframes: (alias/id/model)")
        for airframe in PX4Airframe.get_flying():
            print(f"    {airframe}")