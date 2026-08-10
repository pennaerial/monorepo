from argparse import ArgumentParser

from .command.greeting import GreetingCommand
from .command.mission import MissionCommand
from .command.mode import ModeCommand

EXTENSIONS = {
    "greeting": GreetingCommand,
    "mission": MissionCommand,
    "mode": ModeCommand,
}


# this gets called when the 'pennair' command is ran
def main() -> None:
    parser = ArgumentParser(
        prog="pennair",
        description="Pennair command-line tools.",
        usage="%(prog)s [-h] Call `%(prog)s <command> -h` for more detailed usage. ...",
    )

    # register all sub commands here
    subparsers = parser.add_subparsers(
        title="Commands",
        dest="command",
        required=False,
    )

    # Register command extensions
    for name, extension_cls in EXTENSIONS.items():
        extension = extension_cls()

        command_parser = subparsers.add_parser(
            name,
            help=extension.__doc__,
        )

        extension.add_arguments(
            command_parser,
            "pennair",
        )

        command_parser.set_defaults(
            func=extension.main,
        )

    args = parser.parse_args()

    if not hasattr(args, "func"):
        parser.print_help()
        return

    args.func(args=args)
