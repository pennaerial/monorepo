from argparse import ArgumentParser
from abc import ABC, abstractmethod


class CommandExtension(ABC):
    def add_arguments(
        self,
        parser: ArgumentParser,
        cli_name: str,
    ) -> None:
        """Add this extension's arguments/subcommands."""

    @abstractmethod
    def main(self, *, args):
        """Execute the command."""
