from argparse import ArgumentParser
from typing import override

from ament_index_python.packages import PackageNotFoundError, get_packages_with_prefixes
from pennair_cli.extension import CommandExtension
from pydantic import ValidationError
from vehicle_common.runtime.mission_loader import RuntimeMission, get_mission_path
from vehicle_common.utils import get_available_missions


class MissionCommand(CommandExtension):
    """Prints out general information about all missions. Must provide package and missions names or --all flag"""

    @override
    def add_arguments(self, parser: ArgumentParser, cli_name: str):
        self.parser = parser

        parser.add_argument(
            "--all",
            action="store_true",
            help="Print information about all missions in the package.",
        )

        parser.add_argument(
            "package",
            nargs="?",
            help="Package containing the missions.",
        )

        parser.add_argument(
            "missions",
            nargs="*",
            help="Mission names to print information about.",
        )

    @override
    def main(self, *, args):
        if not args.all and not args.package and not args.missions:
            self.parser.print_help()
            return
        package_names = get_packages_with_prefixes().keys()

        if args.all:  # --all flag overrides everything else
            for pkg in package_names:
                missions = get_available_missions(pkg)
                if missions:
                    print(f"{pkg}:")
                    for mission in missions:
                        print(f"    {mission}")
            return

        # package given, no missions given -> list all missions in package
        # ex pennair mission uav -> lists all uav missions
        if args.package and not args.missions:
            try:
                missions = get_available_missions(args.package)
            except ValueError: # invalid name
                print(f"{args.package}: invalid name")
                return
            except PackageNotFoundError: # not a package
                print(f"{args.package}: package not found")
                return
            if missions:
                print(f"Available missions in \"{args.package}\":")
                for mission in missions:
                    print(f"    {mission}")
            else:
                print(f"{args.package}: no missions found")
            return

        for mission in args.missions:
            mission_path = get_mission_path(mission, args.package)
            try:
                rm = RuntimeMission.load_from_path(mission_path)
            except ValidationError as e:
                print(f"Error: mission at {mission_path} is not a valid mission: {e}")
                return
            except FileNotFoundError as e:
                print(f"{mission_path} does not exist: {e}")
                return

            print(rm)
