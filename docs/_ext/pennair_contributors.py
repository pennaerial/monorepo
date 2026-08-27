# import yaml
from pathlib import Path

from docutils import nodes
from sphinx.application import Sphinx
from sphinx.util.docutils import SphinxDirective


class ContributorsDirective(SphinxDirective):
    has_content = False

    option_spec = {
        "path": str,
    }

    def run(self) -> list[nodes.Node]:

        source_dir = (Path(self.env.app.srcdir) / self.env.docname).parent
        # default to contributors dir if not specified
        contributors_path = source_dir / self.options.get("path", "contributors")
        paragraph = nodes.paragraph()
        paragraph += nodes.Text(str(contributors_path))

        return [paragraph]

def setup(app: Sphinx):
    print("Hello from my extension!")

    app.add_directive(
        "pennair-contributors",
        ContributorsDirective,
    )

    return {
        "version": "0.1.0",
    }
