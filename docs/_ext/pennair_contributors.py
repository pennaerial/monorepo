import yaml
from pathlib import Path

import requests
from docutils import nodes
from docutils.statemachine import StringList
from sphinx.application import Sphinx
from sphinx.util.docutils import SphinxDirective


def get_github_contributions(repo: str) -> dict[str, int]:
    response = requests.get(
        f"https://api.github.com/repos/{repo}/contributors",
        headers={
            "Accept": "application/vnd.github+json",
        },
        timeout=10,
    )
    response.raise_for_status()

    return {
        contributor["login"]: contributor["contributions"]
        for contributor in response.json()
        if contributor.get("login")
    }


# condition to render contributor card based on specified directive role and contributor role
def should_add_contributor(directive_role: str, role: str):
    active = role != "alum"
    return (directive_role == "active" and active) or (directive_role == "alum" and not active)


class ContributorsDirective(SphinxDirective):
    has_content = False

    option_spec = {
        "path": str,
        "repo": str,
        "role": str,
    }

    def run(self) -> list[nodes.Node]:

        source_dir = (Path(self.env.app.srcdir) / self.env.docname).parent
        contributions_list = self._get_contributions(self.options["repo"])
        # default to contributors dir if not specified
        contributors_path = source_dir / self.options.get("path", "contributors")
        directive_role = self.options["role"]

        yaml_files = list(contributors_path.glob("*.yaml"))

        contributors = []
        for path in yaml_files:
            with open(path, "r") as file:
                contributor_data = yaml.safe_load(file)
            github_user = contributor_data.get("github-username", "")
            contributions = contributions_list.get(github_user, 0)

            role = contributor_data.get("role", "member")
            if should_add_contributor(directive_role, role):
                contributors.append(
                    {"github": github_user, "role": role, "contributions": contributions}
                )

        # ordering first priority is by contribution count, and the tiebreaker for same count is alphabetical order
        contributors.sort(key=lambda c: c["github"].casefold())
        contributors.sort(key=lambda c: c["contributions"], reverse=True)

        return self._create_grid(contributors)

    def _get_contributions(self, repo: str) -> dict[str, int]:
        cache = self.env.app.config.pennair_contributors_cache

        if repo in cache:
            return cache[repo]

        contributions = get_github_contributions(repo)

        cache[repo] = contributions

        return contributions

    # create the rst programmatically
    def _create_grid(self, contributors):
        rst = [
            ".. grid:: 2 3 4 5",
            "   :gutter: 2",
            "   :padding: 0",
            "",
        ]

        for contributor in contributors:
            github = contributor["github"]
            role = contributor["role"]

            name_text = f"**{github}**"
            if role.casefold() == "lead":
                name_text += " :iconify:`lucide:bird`"
            rst.extend(
                [
                    "   .. grid-item-card::",
                    f"      :link: https://github.com/{github}",
                    f"      :img-top: https://github.com/{github}.png",
                    f"      :img-alt: {github}",
                    "",
                    f"      {name_text}",
                ]
            )

        container = nodes.container()

        self.state.nested_parse(
            StringList(rst),
            self.content_offset,
            container,
        )

        return container.children


def setup(app: Sphinx):
    app.add_directive(
        "pennair-contributors",
        ContributorsDirective,
    )

    app.add_config_value(
        "pennair_contributors_cache",
        {},
        "env",
    )

    return {
        "version": "0.1.0",
    }
