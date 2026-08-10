from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension


class PennairCommand(CommandExtension):
    """Various pennair related sub-commands"""  # this docstring becomes argparser description in --help

    def add_arguments(self, parser, cli_name, *, argv=None):
        self._subparser = parser  # argparser instance for 'pennair' command

        # get pennair.verb extensions and let them add their arguments
        add_subparsers_on_demand(
            self._subparser,
            cli_name,
            "_verb",
            "pennair.verb",
            required=False,
            argv=argv,
        )

    def main(self, *, parser, args):
        if not hasattr(args, "_verb"):
            # in case no verb was passed
            self._subparser.print_help()
            return 0

        extension = getattr(args, "_verb")

        # call the verb's main method
        return extension.main(args=args)
