"""MkDocs hook for the AFC documentation site.

1. Turns GitHub's inline maths, $`x`$, into the $x$ that pymdownx.arithmatex reads,
   so the same Markdown renders on GitHub and on the site.
2. Points links that leave docs/ (source files, LICENSE, ...) at the file on GitHub.
3. Turns a line `<!-- viz: NAME HEIGHT -->` into an embedded visualiser from
   docs/interactive/NAME/index.html. On GitHub the marker is an invisible comment.
"""

from __future__ import annotations

import logging
import posixpath
import re

log = logging.getLogger("mkdocs.hooks.site")

BRANCH = "main"

GITHUB_MATH = re.compile(r"\$`([^`\n]+?)`\$")
MD_LINK = re.compile(r"(!?\[[^\]]*\]\()([^)\s]+)(\))")
VIZ = re.compile(r"^<!--\s*viz:\s*([a-z0-9-]+)(?:\s+(\d+))?\s*-->\s*$", re.MULTILINE)

_repo_url = ""


def _rewrite_outside_link(target: str, page: str) -> str:
    """A link in a docs/ page that climbs out of docs/ goes to the file on GitHub."""
    if re.match(r"^[a-z]+:", target) or target.startswith(("#", "/")):
        return target
    path, _, anchor = target.partition("#")
    resolved = posixpath.normpath(posixpath.join("docs", posixpath.dirname(page), path))
    if resolved.startswith("docs/") or resolved == "docs":
        return target
    return f"{_repo_url}/blob/{BRANCH}/{resolved}" + (f"#{anchor}" if anchor else "")


def _viz(match: re.Match, page, files) -> str:
    name, height = match.group(1), match.group(2) or "600"
    target = f"interactive/{name}/index.html"
    if files.get_file_from_path(target) is None:
        # A warning fails a --strict build (CI) but still lets `mkdocs serve` show the page.
        log.warning(f"{page.file.src_uri}: no visualiser at docs/{target}")
        return f'<p class="afc-viz-missing">Visualiser <code>{name}</code> is not built yet.</p>'
    depth = page.url.count("/")
    url = "../" * depth + f"interactive/{name}/"
    return (
        f'<div class="afc-viz">'
        f'<iframe src="{url}" style="height:{height}px" loading="lazy" title="{name}"></iframe>'
        f'<a class="afc-viz-open" href="{url}" target="_blank" rel="noopener">Open full screen</a>'
        f"</div>"
    )


# ---- MkDocs events --------------------------------------------------------------------------


def on_config(config, **kwargs):
    global _repo_url
    _repo_url = (config.repo_url or "").rstrip("/")
    return config


FENCED_BLOCK = re.compile(r"(^ {0,3}(`{3,}|~{3,})[^\n]*\n.*?\n {0,3}\2[ \t]*$)", re.MULTILINE | re.DOTALL)


def _transform_outside_fences(markdown: str, fn) -> str:
    """Apply fn only to markdown outside fenced code blocks."""
    out = []
    pos = 0
    for match in FENCED_BLOCK.finditer(markdown):
        out.append(fn(markdown[pos:match.start()]))
        out.append(match.group(0))
        pos = match.end()
    out.append(fn(markdown[pos:]))
    return "".join(out)


def on_page_markdown(markdown, page, config, files, **kwargs):
    src = page.file.src_uri

    def transform(chunk: str) -> str:
        chunk = GITHUB_MATH.sub(lambda m: f"${m.group(1)}$", chunk)
        chunk = VIZ.sub(lambda m: _viz(m, page, files), chunk)
        chunk = MD_LINK.sub(
            lambda m: m.group(1) + _rewrite_outside_link(m.group(2), src) + m.group(3),
            chunk,
        )
        return chunk

    return _transform_outside_fences(markdown, transform)
