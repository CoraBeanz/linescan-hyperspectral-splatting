#!/usr/bin/env python3
"""Check the relative links and images in the repo's Markdown files.

Every link or image that points into the repo has to reach a tracked file or
folder, and a #anchor has to match a heading (or an id) in the Markdown file it
points at, the way GitHub turns headings into anchors. Web links aren't fetched.

    python .github/scripts/check_links.py
"""
import re
import subprocess
import sys
import unicodedata
from pathlib import Path
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[2]

FENCE = re.compile(r"^ {0,3}(`{3,}|~{3,})")
COMMENT = re.compile(r"<!--.*?-->", re.S)
INLINE_CODE = re.compile(r"(`+)(?!`).*?(?<!`)\1(?!`)")
MD_LINK = re.compile(r"\[(?:[^\[\]]|\[[^\]]*\])*\]\(\s*<?([^)\s>]+)>?(?:\s+\"[^\"]*\")?\s*\)")
HTML_LINK = re.compile(r"""\b(?:href|src)\s*=\s*["']([^"']+)["']""", re.I)
HTML_ID = re.compile(r"""<[^>]*\b(?:id|name)\s*=\s*["']([^"']+)["']""", re.I)
HEADING = re.compile(r"^ {0,3}#{1,6}\s+(.*?)(?:\s+#+)?\s*$")
WEB = re.compile(r"^(?:[a-z][a-z0-9+.-]*:|//)", re.I)   # http:, https:, mailto:, data:, //host


def tracked():
    files = set(subprocess.check_output(["git", "ls-files"], cwd=ROOT, text=True).splitlines())
    dirs = {"."}
    for f in files:
        p = Path(f).parent
        while str(p) != ".":
            dirs.add(p.as_posix())
            p = p.parent
    return files, dirs


def prose_lines(text):
    """(line number, line, line without code spans) for each line outside fenced code
    and HTML comments."""
    text = COMMENT.sub(lambda m: "\n" * m.group(0).count("\n"), text)
    fence = None
    for n, line in enumerate(text.splitlines(), 1):
        m = FENCE.match(line)
        if fence:
            if m and m.group(1)[0] == fence[0] and len(m.group(1)) >= len(fence):
                fence = None
            continue
        if m:
            fence = m.group(1)
            continue
        yield n, line, INLINE_CODE.sub("", line)


def slug(heading):
    """GitHub's anchor for a heading: the rendered text, lower case, punctuation
    dropped, each space a hyphen."""
    text = re.sub(r"<[^>]+>", "", heading)
    text = re.sub(r"!?\[((?:[^\[\]]|\[[^\]]*\])*)\]\([^)]*\)", r"\1", text)
    text = re.sub(r"(\*{1,3}|_{1,3})(?=\S)(.+?)(?<=\S)\1", r"\2", text)
    text = text.replace("`", "")
    keep = (c for c in text.lower() if c in " -_" or unicodedata.category(c)[0] in "LNM")
    return "".join(keep).replace(" ", "-")


_anchors = {}


def anchors(md):
    if md not in _anchors:
        seen, found = {}, set()
        for _, raw, line in prose_lines(md.read_text(encoding="utf-8")):
            found.update(a.lower() for a in HTML_ID.findall(line))
            m = HEADING.match(raw)
            if m:
                s = slug(m.group(1))
                found.add(s if s not in seen else "%s-%d" % (s, seen[s]))
                seen[s] = seen.get(s, 0) + 1
        _anchors[md] = found
    return _anchors[md]


def main():
    files, dirs = tracked()
    errors = []
    for name in sorted(f for f in files if f.endswith(".md")):
        md = ROOT / name
        for n, _, line in prose_lines(md.read_text(encoding="utf-8")):
            for target in MD_LINK.findall(line) + HTML_LINK.findall(line):
                if WEB.match(target):
                    continue
                path, _, anchor = target.partition("#")
                path = unquote(path.split("?", 1)[0])
                if not path:
                    dest = md
                elif path.startswith("/"):
                    dest = ROOT / path.lstrip("/")
                else:
                    dest = md.parent / path
                try:
                    rel = dest.resolve().relative_to(ROOT.resolve()).as_posix()
                except ValueError:
                    errors.append("%s:%d: %s points outside the repo" % (name, n, target))
                    continue
                if rel not in files and rel not in dirs:
                    errors.append("%s:%d: %s: no tracked file or folder %s" % (name, n, target, rel))
                elif anchor and rel.endswith(".md") and unquote(anchor).lower() not in anchors(ROOT / rel):
                    errors.append("%s:%d: %s: no heading for #%s in %s" % (name, n, target, anchor, rel))
    for e in errors:
        print(e)
    print("%d broken link%s" % (len(errors), "" if len(errors) == 1 else "s"))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
