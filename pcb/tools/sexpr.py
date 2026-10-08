"""Small S-expression reader and writer for KiCad files.

KiCad's schematic, board, symbol and footprint files are all S-expressions:
nested lists in parentheses, with quoted strings and bare words (atoms).
build_boards.py writes its files through dump(), then lets kicad-cli re-save
them, so what lands in git is exactly what KiCad itself would write.
"""
import re


class A(str):
    """A bare word, written without quotes (pin types, yes/no flags, ...)."""


_TOKEN = re.compile(r'\s*(?:(\()|(\))|"((?:[^"\\]|\\.)*)"|([^\s()"]+))', re.S)


def parse(text):
    """Text -> nested lists. Quoted strings come back as str, bare words as A."""
    stack = [[]]
    for m in _TOKEN.finditer(text):
        if m.group(1):
            stack.append([])
        elif m.group(2):
            done = stack.pop()
            stack[-1].append(done)
        elif m.group(3) is not None:
            stack[-1].append(m.group(3).replace('\\"', '"').replace("\\\\", "\\"))
        else:
            stack[-1].append(A(m.group(4)))
    return stack[0][0]


def find(node, key):
    """First child list whose head is key."""
    for c in node:
        if isinstance(c, list) and c and c[0] == key:
            return c
    return None


def find_all(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]


def walk(node, key):
    """Every list anywhere below node whose head is key."""
    for c in node:
        if isinstance(c, list):
            if c and c[0] == key:
                yield c
            yield from walk(c, key)


def num(v):
    s = ("%.4f" % v).rstrip("0").rstrip(".")
    return "0" if s in ("", "-0") else s


def atom(v):
    if isinstance(v, A):
        return str(v)
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, (int, float)):
        return num(float(v))
    s = str(v).replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
    return '"%s"' % s


def dump(node, depth=0):
    """Nested lists -> text, one list per line, tab-indented like KiCad."""
    ind = "\t" * depth
    if not any(isinstance(c, list) for c in node):
        return ind + "(" + " ".join(atom(c) for c in node) + ")"
    i = 0
    head = []
    while i < len(node) and not isinstance(node[i], list):
        head.append(atom(node[i]))
        i += 1
    lines = [ind + "(" + " ".join(head)]
    for c in node[i:]:
        lines.append(dump(c, depth + 1) if isinstance(c, list) else ind + "\t" + atom(c))
    lines.append(ind + ")")
    return "\n".join(lines)


def block(text, start):
    """The balanced (...) that opens at text[start], strings respected."""
    depth = 0
    i = start
    in_str = False
    while i < len(text):
        ch = text[i]
        if in_str:
            if ch == "\\":
                i += 1
            elif ch == '"':
                in_str = False
        elif ch == '"':
            in_str = True
        elif ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
        i += 1
    raise ValueError("unbalanced S-expression")


def top_symbol(lib_text, name):
    """Raw text of one top-level symbol in a .kicad_sym library."""
    m = re.search(r'^\t\(symbol "%s"\s*$' % re.escape(name), lib_text, re.M)
    if not m:
        raise KeyError("symbol %s not found" % name)
    return block(lib_text, m.start() + 1)
