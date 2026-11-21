"""MkDocs hook for the AFC documentation site.

1. Builds the User Guide pages from README.md, so the README stays the one copy of the
   player manual. USER_GUIDE below says which README sections go on which page.
2. Turns GitHub's inline maths, $`x`$, into the $x$ that pymdownx.arithmatex reads,
   so the same Markdown renders on GitHub and on the site.
3. Points links that leave docs/ (source files, LICENSE, ...) at the file on GitHub.
"""

from __future__ import annotations

import posixpath
import re
from pathlib import Path

from mkdocs.structure.files import File
from pymdownx.slugs import slugify as _make_slugify

slugify = _make_slugify(case="lower")  # the same GitHub-style ids as toc in mkdocs.yml

ROOT = Path(__file__).resolve().parents[2]
README = ROOT / "README.md"
BRANCH = "main"

# (page, title, README sections). A section is an H2 or H3 heading of the README and runs to the
# next heading of the same or a higher level. "#" is the text above the first H2. A page with one
# section takes that section's heading as its title.
USER_GUIDE = [
    ("user-guide/index.md", "User Guide", ["#", "The AFC console at a glance"]),
    ("user-guide/installation.md", "Installation", ["Installation", "Dependencies", "Build dependencies"]),
    ("user-guide/console.md", None, ["AFC Console"]),
    (
        "user-guide/maneuvers.md",
        "Maneuvers and transfers",
        ["Maneuver Quick-Tools", "Flyby Targeting", "Multi-Pass Burns", "Hyperbolic Targets"],
    ),
    (
        "user-guide/burns-and-staging.md",
        "Burns and staging",
        ["RCS Translation Burns", "Automatic Staging", "Automatic Burn Removal"],
    ),
    ("user-guide/powered-guidance.md", "Powered guidance", ["Powered Guidance", "Launch to a moon"]),
    (
        "user-guide/about.md",
        "Compatibility and credits",
        ["Mod compatibility", "Credits", "License", "Community", "Check out my other mods"],
    ),
]

HEADING = re.compile(r"^(#{1,6})\s+(.*?)\s*$")
FENCE = re.compile(r"^\s*(```|~~~)")
GITHUB_MATH = re.compile(r"\$`([^`\n]+?)`\$")
MD_LINK = re.compile(r"(!?\[[^\]]*\]\()([^)\s]+)(\))")
HTML_SRC = re.compile(r'(<img\b[^>]*\bsrc=")([^"]+)(")')

_repo_url = ""
_generated: dict[str, str] = {}  # page src_uri -> markdown
_anchor_page: dict[str, str] = {}  # README anchor -> page src_uri


def _readme_sections(text: str) -> dict[str, tuple[int, list[str]]]:
    """Split the README into {heading: (level, lines)}; "#" is the preamble under the H1."""
    sections: dict[str, tuple[int, list[str]]] = {"#": (1, [])}
    current = "#"
    in_fence = False
    for line in text.splitlines():
        if FENCE.match(line):
            in_fence = not in_fence
        m = None if in_fence else HEADING.match(line)
        if m and len(m.group(1)) == 1:
            continue  # the README title
        if m and len(m.group(1)) in (2, 3):
            current = m.group(2)
            sections[current] = (len(m.group(1)), [line])
            continue
        sections[current][1].append(line)
    return sections


def _shift(lines: list[str], by: int) -> list[str]:
    """Move every heading by `by` levels (negative promotes), outside code fences."""
    out, in_fence = [], False
    for line in lines:
        if FENCE.match(line):
            in_fence = not in_fence
        m = None if in_fence else HEADING.match(line)
        if m:
            level = min(6, max(1, len(m.group(1)) + by))
            line = "#" * level + " " + m.group(2)
        out.append(line)
    return out


def _build_user_guide() -> None:
    sections = _readme_sections(README.read_text(encoding="utf-8"))
    _generated.clear()
    _anchor_page.clear()
    for page, title, names in USER_GUIDE:
        body: list[str] = []
        single = len(names) == 1
        for name in names:
            if name not in sections:
                raise KeyError(f"docs/hooks/site.py: README has no section '{name}' for {page}")
            level, lines = sections[name]
            # A single-section page shows its heading as the H1; otherwise sections sit at H2.
            shift = (1 - level) if single else (2 - level)
            body += _shift(lines, shift)
            for line in lines:
                m = HEADING.match(line)
                if m:
                    _anchor_page.setdefault(slugify(m.group(2), "-"), page)
        if not single:
            body = [f"# {title}", ""] + body
        _generated[page] = "\n".join(body).strip() + "\n"


def _github(path: str) -> str:
    return f"{_repo_url}/blob/{BRANCH}/{path}"


def _rewrite_readme_link(target: str, page: str, raw_html_depth: int | None) -> str:
    """A link written relative to the repo root, as the README has it, seen from `page`."""
    if re.match(r"^[a-z]+:", target) or target.startswith("//"):
        return target
    path, _, anchor = target.partition("#")
    if not path:  # an anchor in the README
        owner = _anchor_page.get(anchor)
        if owner and owner != page:
            return posixpath.relpath(owner, posixpath.dirname(page)) + "#" + anchor
        return target
    if path.startswith("docs/") or path.startswith("images/"):
        in_site = path[len("docs/"):] if path.startswith("docs/") else path
        if raw_html_depth is not None:  # raw HTML is not rewritten by MkDocs: use the page URL
            rel = "../" * raw_html_depth + in_site
        else:
            rel = posixpath.relpath(in_site, posixpath.dirname(page))
        return rel + (f"#{anchor}" if anchor else "")
    return _github(path) + (f"#{anchor}" if anchor else "")


def _rewrite_outside_link(target: str, page: str) -> str:
    """A link in a docs/ page that climbs out of docs/ goes to the file on GitHub."""
    if re.match(r"^[a-z]+:", target) or target.startswith(("#", "/")):
        return target
    path, _, anchor = target.partition("#")
    resolved = posixpath.normpath(posixpath.join("docs", posixpath.dirname(page), path))
    if resolved.startswith("docs/") or resolved == "docs":
        return target
    return _github(resolved) + (f"#{anchor}" if anchor else "")


# ---- MkDocs events --------------------------------------------------------------------------


def on_config(config, **kwargs):
    global _repo_url
    _repo_url = (config.repo_url or "").rstrip("/")
    _build_user_guide()
    config.watch.append(str(README))
    return config


def on_files(files, config, **kwargs):
    for page, content in _generated.items():
        files.append(File.generated(config, page, content=content))
    # The README's pictures live in images/ at the repo root; serve them at /images/.
    for path in sorted((ROOT / "images").rglob("*")):
        if path.is_file():
            rel = path.relative_to(ROOT).as_posix()
            files.append(File(rel, str(ROOT), config.site_dir, config.use_directory_urls))
    return files


def on_page_markdown(markdown, page, config, files, **kwargs):
    src = page.file.src_uri
    markdown = GITHUB_MATH.sub(lambda m: f"${m.group(1)}$", markdown)
    if src in _generated:
        depth = page.url.count("/") if config.use_directory_urls else src.count("/")
        markdown = MD_LINK.sub(
            lambda m: m.group(1) + _rewrite_readme_link(m.group(2), src, None) + m.group(3), markdown
        )
        markdown = HTML_SRC.sub(
            lambda m: m.group(1) + _rewrite_readme_link(m.group(2), src, depth) + m.group(3), markdown
        )
        page.edit_url = f"{_repo_url}/edit/{BRANCH}/README.md"
    else:
        markdown = MD_LINK.sub(
            lambda m: m.group(1) + _rewrite_outside_link(m.group(2), src) + m.group(3), markdown
        )
    return markdown
