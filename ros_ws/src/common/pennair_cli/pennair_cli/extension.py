from abc import ABC, abstractmethod
from argparse import ArgumentParser


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
