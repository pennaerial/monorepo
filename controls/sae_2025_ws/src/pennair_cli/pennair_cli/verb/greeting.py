from ros2cli.verb import VerbExtension


def link(text: str, url: str) -> str:
    return f"\033]8;;{url}\033\\{text}\033]8;;\033\\"


GREETING_MESSAGE = rf"""

⠀⠀⠀⠀⠀⠀⢀⣠⠴⠒⠒⠒⠒⠒⠶⠦⠤⠴⠒⠚⠉⣰⠟⠁⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⢀⡞⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠒⡿⠃⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⢠⡿⣤⣄⠀⠀⠀⠀⠀⢀⣤⣄⠀⠀⠀⣰⠞⠁⠀⠀⠀⢠⣤⠀⠀⠀⠀
⠀⠀⢠⡟⠸⣿⡿⢀⠤⢄⠀⠐⣷⣿⣿⡷⠀⠀⢻⠀⠀⠀⢀⡴⠋⠘⡇⠀⠀⠀
⠀⠀⢸⠈⠲⡬⢠⡏⠀⢀⡷⠀⠨⣭⠭⠖⣇⠀⣾⠀⠀⡴⠋⠀⠀⠀⣧⠀⠀⠀
⠀⠀⢸⣷⠞⠁⠀⠳⠖⠋⠀⠀⠀⠙⠶⠶⠃⡼⢿⣠⠎⠀⠀⠀⠀⠀⣿⠀⠀⠀
⠀⠀⢸⣇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡾⠃⠀⠀⠀⠀⠀⠀⡟⠀⠀⠀
⠀⠀⡼⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠹⡄⠀⠀⠀⠀⠀⢠⠃⠀⠀⠄
⢀⣰⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠐⠉⠉⠉⠉⠲⢄⡀⢀⣠⡷⠀⠀⢠
⢸⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠓⠧⠤⢾⢀⠏
⢸⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢨⠏⠀
⢸⠀⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⣇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡏⠀⠀
⠈⢷⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠢⡀⠀⠀⠀⠀⠀⠀⢀⣤⡏⠀⠀⠀
⠀⠈⣧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠑⠲⠶⠶⠖⠊⠁⣴⠇⠀⠀⠀
⠀⠀⠘⢧⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣾⠁⠀⠀⠀⠀
⠀⠀⠀⠈⠻⢦⣀⠀⠀⠀⠀⠀⠀⣠⣎⡀⡀⠀⠀⣀⣰⢶⠶⠚⠁⠀⠀⠀⠀⠀
⠀⠀⠀⠀⣰⠛⠉⡙⠛⢛⣷⠖⠒⢖⣾⠟⢛⠛⠺⣿⣏⠁⠀⠀⠀⠀⠀⠀⠀⠀
______                 ___  _______
| ___ \               / _ \(_) ___ \
| |_/ /__ _ __  _ __ / /_\ \_| |_/ /
|  __/ _ \ '_ \| '_ \|  _  | |    /
| | |  __/ | | | | | | | | | | |\ \
\_|  \___|_| |_|_| |_\_| |_/_\_| \_|

Greetings from the PennAiR Software Team!

{link("GitHub: https://github.com/pennaerial/monorepo", "https://github.com/pennaerial/monorepo")}
{link("Docs: https://pennaerial.github.io/monorepo", "https://pennaerial.github.io/monorepo")}
"""


class GreetingVerb(VerbExtension):
    """Prints out a greeting from the PennAiR Software Team."""

    def main(self, *, args):
        print(GREETING_MESSAGE)
