from argparse import ArgumentParser


# this gets called when the 'pennair' command is ran
def main() -> None:
    parser = ArgumentParser(
        prog="pennair",
        description="Pennair command-line tools.",
    )

    # subparsers = parser.add_subparsers(dest="command")

    args = parser.parse_args()

    if not hasattr(args, "func"):
        parser.print_help()
        return

    args.func(args)
