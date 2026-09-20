from argparse import ArgumentParser
from importlib import import_module

COMMANDS = {
    "airframe": (".command.airframe", "AirframeCommand", "Prints out available UAV airframes."),
    "greeting": (".command.greeting", "GreetingCommand", "Prints a greeting."),
    "mission": (".command.mission", "MissionCommand", "Prints out general information about all missions."),
    "mode": (".command.mode", "ModeCommand", "Prints out general information about all registered modes."),
    "world": (".command.world", "WorldCommand", "Prints out available worlds."),
}


# this gets called when the 'pennair' command is ran
def main() -> None:
    parser = ArgumentParser(
        prog="pennair",
        description="Pennair command-line tools.",
        usage="%(prog)s [OPTIONS] COMMAND",
    )

    # register all sub commands here
    subparsers = parser.add_subparsers(
        title="Commands",
        dest="command",
        required=False,
    )
    command_parsers = {} # store the command parsers so we can add arguments to them later
    for name, (_, _, description) in COMMANDS.items():
        command_parser = subparsers.add_parser(
            name,
            description=description,
            help=description,
            usage=f"pennair {name} [OPTIONS] COMMAND",
        )
        command_parsers[name] = command_parser

    # Parse just far enough to identify the command before importing its extension.
    # Avoids importing all extensions when its not necessary
    preliminary_args, _ = parser.parse_known_args()
    if preliminary_args.command is None:
        parser.print_help()
        return

    module_name, class_name, _ = COMMANDS[preliminary_args.command]
    extension_module = import_module(module_name, package=__package__)
    extension = getattr(extension_module, class_name)()
    command_parser = command_parsers[preliminary_args.command]
    extension.add_arguments(command_parser, "pennair")
    command_parser.set_defaults(func=extension.main)

    args = parser.parse_args()

    if not hasattr(args, "func"):
        parser.print_help()
        return

    args.func(args=args)
