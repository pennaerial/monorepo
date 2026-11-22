"""Page-owned status banners, accessible navigation badges, and a live status list."""

from datetime import date
from html import escape
from html.parser import HTMLParser
from urllib.parse import urljoin, urlsplit

from docutils import nodes
from sphinx.util.docutils import SphinxDirective
from sphinx.directives.other import TocTree


class PythonAPITocTree(TocTree):
    """Omit the generated API entry before toctree collection during fast builds."""

    def run(self):
        if self.env.config.skip_autoapi_build:
            return []
        return super().run()


class StatusList(nodes.General, nodes.Element):
    pass


class StatusListDirective(SphinxDirective):
    def run(self):
        return [StatusList()]


class KeyListDirective(SphinxDirective):
    def run(self):
        return [StatusList(kind="key")]


def evolving_pages(env):
    return {
        docname: metadata.get("status-reason", "This page is under active development.")
        for docname, metadata in env.metadata.items()
        if metadata.get("doc-status") == "evolving"
    }


def review_status(env, docname):
    metadata = env.metadata.get(docname, {})
    reviewer = str(metadata.get("verified-by", "")).strip()
    review_date = str(metadata.get("verified-on", "")).strip()
    try:
        date.fromisoformat(review_date)
        valid_date = True
    except ValueError:
        valid_date = False
    verified = str(metadata.get("human-verified", "")).lower() == "true" and bool(reviewer) and valid_date
    return verified, reviewer, review_date


def page_statuses(env):
    result = {}
    evolving = evolving_pages(env)
    for name in env.found_docs:
        if name not in env.titles:
            continue
        verified, reviewer, date = review_status(env, name)
        result[name] = {
            "key": env.metadata.get(name, {}).get("doc-priority") == "key",
            "key_reason": env.metadata.get(name, {}).get("key-reason", "Read this page to understand the subsystem before changing it."),
            "reason": evolving.get(name),
            "verified": verified,
            "reviewer": reviewer,
            "date": date,
        }
    return result


def resolve_status(app, doctree, docname):
    statuses = page_statuses(app.env)
    state = statuses.get(docname)
    section = next(iter(doctree.findall(nodes.section)), None)
    if state and section is not None:
        banner = nodes.admonition(classes=["docs-review-banner"])
        if state["verified"]:
            banner["classes"].append("docs-review-verified")
            banner += nodes.title(text="Human verified")
            banner += nodes.paragraph(text=f'Reviewed by {state["reviewer"]} on {state["date"]}. Review applies to this page, not every linked subsystem.')
        else:
            banner += nodes.title(text="Not human verified")
            banner += nodes.paragraph(text="This page has not been recorded as reviewed by a club member. Check its instructions and assumptions before relying on them.")
        section.insert(1, banner)
        if state["key"]:
            key = nodes.admonition(classes=["docs-key-banner"])
            key += nodes.title(text="Key reading")
            key += nodes.paragraph(text=state["key_reason"])
            section.insert(1, key)
        if state["reason"]:
            evolving = nodes.admonition(classes=["docs-status-banner"])
            evolving += nodes.title(text="In progress / likely to change")
            evolving += nodes.paragraph(text=state["reason"])
            section.insert(2, evolving)

    for placeholder in list(doctree.findall(StatusList)):
        listing = nodes.bullet_list()
        key_list = placeholder.get("kind") == "key"
        for target, state in sorted(statuses.items()):
            if not (state["key"] if key_list else state["reason"]):
                continue
            title = app.env.titles[target].astext()
            uri = app.builder.get_relative_uri(docname, target)
            label = title + (" · Key" if state["key"] else "") + (" · WIP" if state["reason"] else "")
            label += " · Human verified" if state["verified"] else " · Not human verified"
            css = ["docs-reading-key"] if key_list else ["docs-status-evolving"]
            link = nodes.reference("", label, refuri=uri, classes=css)
            paragraph = nodes.paragraph()
            paragraph += link
            paragraph += nodes.Text(" — " + (state["key_reason"] if key_list else state["reason"]))
            listing += nodes.list_item("", paragraph)
        placeholder.replace_self(listing)


class StatusNavigation(HTMLParser):
    """Annotate generated toctree links without depending on theme internals."""

    def __init__(self, page_uri, statuses):
        super().__init__(convert_charrefs=False)
        self.base = "https://docs.invalid/" + page_uri
        self.statuses = statuses
        self.output = []
        self.active = False

    def handle_starttag(self, tag, attrs):
        attributes = dict(attrs)
        target = urlsplit(urljoin(self.base, attributes.get("href", "")))
        state = self.statuses.get(target.path.lstrip("/")) if target.netloc == "docs.invalid" else None
        if tag == "a" and state is not None:
            css = "docs-status-evolving" if state["reason"] else ""
            css += " docs-human-verified" if state["verified"] else " docs-not-verified"
            if state["key"]:
                css += " docs-reading-key"
            attributes["class"] = (attributes.get("class", "") + " " + css).strip()
            labels = []
            if state["key"]:
                labels.append("Key reading: " + state["key_reason"])
            if state["reason"]:
                labels.append("In progress: " + state["reason"])
            labels.append(f'Human verified by {state["reviewer"]} on {state["date"]}' if state["verified"] else "Not human verified")
            attributes["title"] = "; ".join(labels)
            rendered = " ".join(
                key if value is None else f'{key}="{escape(value, quote=True)}"'
                for key, value in attributes.items()
            )
            self.output.append(f"<{tag} {rendered}>")
            self.active = state
        else:
            self.output.append(self.get_starttag_text())

    def handle_endtag(self, tag):
        if tag == "a" and self.active:
            state = self.active
            if state["key"] or state["reason"]:
                self.output.append('<span class="docs-nav-badges">')
            if state["key"]:
                self.output.append('<span class="docs-key-badge">Key</span>')
            if state["reason"]:
                self.output.append('<span class="docs-status-badge">WIP</span>')
            if state["key"] or state["reason"]:
                self.output.append('</span>')
            label = "Human verified" if state["verified"] else "Not human verified"
            self.output.append(f'<span class="docs-review-badge">{label}</span>')
            self.active = False
        self.output.append(f"</{tag}>")

    def handle_startendtag(self, tag, attrs):
        self.output.append(self.get_starttag_text())

    def handle_data(self, data):
        self.output.append(data)

    def handle_entityref(self, name):
        self.output.append(f"&{name};")

    def handle_charref(self, name):
        self.output.append(f"&#{name};")

    def handle_comment(self, data):
        self.output.append(f"<!--{data}-->")


def page_context(app, pagename, templatename, context, doctree):
    original = context.get("toctree")
    if original is None:
        return
    statuses = {app.builder.get_target_uri(name): state for name, state in page_statuses(app.env).items()}

    def status_toctree(*args, **kwargs):
        markup = original(*args, **kwargs)
        if not markup:
            return markup
        parser = StatusNavigation(app.builder.get_target_uri(pagename), statuses)
        parser.feed(str(markup))
        parser.close()
        return "".join(parser.output)

    context["toctree"] = status_toctree


def setup(app):
    app.add_directive("docs-status-list", StatusListDirective)
    app.add_directive("docs-key-list", KeyListDirective)
    app.add_directive("docs-python-api", PythonAPITocTree)
    app.add_css_file("docs-status.css")
    app.connect("doctree-resolved", resolve_status)
    app.connect("html-page-context", page_context)
    # Status is cross-page data: rewrite navigation and the map when metadata changes.
    app.connect("env-updated", lambda app, env: sorted(env.found_docs))
    return {"version": "1.0", "parallel_read_safe": True, "parallel_write_safe": True}
