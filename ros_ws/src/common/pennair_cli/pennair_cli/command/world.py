from argparse import ArgumentParser
from pathlib import Path
from typing import override

from pennair_cli.extension import CommandExtension
from sim.utils import get_available_worlds
from vehicle_common.env import require_env

PENNAIR_GZ_MODELS_PATH = require_env("PENNAIR_GZ_MODELS_PATH")
GZ_WORLDS_PATH = Path(PENNAIR_GZ_MODELS_PATH) / "worlds"


class WorldCommand(CommandExtension):
    """Prints out available worlds."""

    @override
    def add_arguments(self, parser: ArgumentParser, cli_name: str):
        self.parser = parser

        subparsers = parser.add_subparsers(
            title="Commands",
            dest="world_command",
            required=False,
        )

        ls_parser = subparsers.add_parser("ls", help="List available worlds.")
        ls_parser.set_defaults(func=self.ls)

    @override
    def main(self, *, args):
        self.parser.print_help()

    def ls(self, *, args) -> None:
        print("Available worlds:")
        for world in get_available_worlds(GZ_WORLDS_PATH):
            print(f"    {world}")
