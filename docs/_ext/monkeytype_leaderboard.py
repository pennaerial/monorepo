import yaml
from pathlib import Path

from docutils import nodes
from docutils.statemachine import StringList
from sphinx.util.docutils import SphinxDirective


class MonkeytypeLeaderboard(SphinxDirective):
    has_content = False

    option_spec = {
        "path": str,
    }

    def run(self) -> list[nodes.Node]:
        source_dir = (Path(self.env.app.srcdir) / self.env.docname).parent
        # default to contributors dir if not specified
        contributors_path = source_dir / self.options.get("path", "contributors")
        yaml_files = list(contributors_path.glob("*.yaml"))

        contributors = []
        for path in yaml_files:
            with open(path, "r") as file:
                contributor_data = yaml.safe_load(file)
            github_user = contributor_data.get("github-username", "")
            monkeytype = int(contributor_data.get("monkeytype", 0))

            if monkeytype == 0:  # skip ppl with no monkeytype entry
                continue

            contributors.append({"github": github_user, "monkeytype": monkeytype})

        # ordering first priority is by monkeytype wpm, and the tiebreaker for same count is alphabetical order
        contributors.sort(key=lambda c: c["github"].casefold())
        contributors.sort(key=lambda c: c["monkeytype"], reverse=True)

        return self._create_leaderboard(contributors)

    # create the rst programmatically
    def _create_leaderboard(self, contributors) -> list[nodes.Node]:
        rst = [
            ".. card:: :iconify:`lucide:trophy` Monkeytype Leaderboard",
            "   :class-card: sd-border-primary",
            "   :shadow: sm",
            "",
        ]

        if contributors:
            rst.extend(
                [
                    "   .. list-table::",
                    "      :header-rows: 1",
                    "      :widths: 15 60 25",
                    "      :width: 100%",
                    "",
                    "      * - Rank",
                    "        - Contributor",
                    "        - WPM",
                ]
            )

            previous_score = None
            rank = 0

            for position, contributor in enumerate(contributors, start=1):
                github = contributor["github"]
                score = contributor["monkeytype"]

                if score != previous_score:
                    rank = position
                previous_score = score

                rst.extend(
                    [
                        f"      * - **{rank}**",
                        f"        - `{github} <https://github.com/{github}>`__",
                        f"        - **{score:,}**",
                    ]
                )

            rst.extend(
                [
                    "",
                    "   +++",
                    "   Self-reported WPM. Equal scores share a rank.",
                    "",
                ]
            )
        else:
            rst.extend(
                [
                    "   No scores yet. Add your ``monkeytype`` WPM to your",
                    "   contributor YAML file to join the leaderboard.",
                    "",
                ]
            )

        container = nodes.container()
        self.state.nested_parse(
            StringList(rst),
            self.content_offset,
            container,
        )
        return container.children
