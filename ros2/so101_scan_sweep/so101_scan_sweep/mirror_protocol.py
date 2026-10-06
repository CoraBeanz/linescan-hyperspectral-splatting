"""The scan-mirror ESP32's serial protocol, version 1. firmware/PROTOCOL.md has all of it.

Plain ASCII lines at 921600 baud. The host sends `VERB key=value ... id=<n>`, and every command
gets exactly one reply, in the order sent, carrying the same id:

    OK VERB key=value ... id=<n>
    ERR VERB code=<code> id=<n> msg=<text to the end of the line>

The ESP32 also sends events when something happens: `EV NAME key=value ...`. Positions are
microsteps (6400 per mirror turn at the default 1/32 microstepping), and after HOME position 0
is the mirror's 45 deg rest. Times are the ESP32's own microsecond clock.

What scan_mirror_bridge uses:

    INFO                       OK INFO fw= proto= usteps= full_steps= ...
    PING                       OK PING t=
    STATUS                     OK STATUS state= pos= homed= ...
    ENABLE                     OK ENABLE drv=        (the motor is off after every start)
    HOME                       OK HOME, then EV HOMED pos= t= width=  or  EV HOME_FAILED reason=
    MOVE pos=                  OK MOVE pos=, then EV MOVED pos= t=
    SCAN mode=stare start= step= lines= period=
                               OK SCAN t0= ..., then EV LINE n= t= pos= ready= for each line,
                               then EV SCAN_DONE lines= t= pos= aborted=
    STOP                       OK STOP, then EV SCAN_DONE aborted=1 if a scan was running, and
                               EV STOPPED pos= t=

and, at any time, EV BOOT (it has just started: motor off, not homed), EV FAULT and EV WARN.
Lines that start with anything else, like the boot ROM's output after a reset, aren't protocol.
"""

import re
from dataclasses import dataclass, field

PROTOCOL_VERSION = 1
BAUD = 921600
# ESP32 states in which it won't take a new HOME, SCAN (or, but for "moving", MOVE)
BUSY_STATES = ("moving", "stopping", "homing", "scanning")


class ProtocolError(ValueError):
    pass


def _value(v):
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, float):
        text = ("%.6f" % v).rstrip("0").rstrip(".")
        return "0" if text in ("", "-0") else text
    text = str(v)
    if not text or any(c.isspace() for c in text):
        raise ProtocolError("value %r is empty or has spaces" % (v,))
    return text


def command(verb, **fields):
    """command("MOVE", pos=12, id=3) -> "MOVE pos=12 id=3\\n". Fields that are None are left out."""
    words = [verb.upper()] + ["%s=%s" % (k, _value(v)) for k, v in fields.items() if v is not None]
    return " ".join(words) + "\n"


@dataclass(frozen=True)
class Message:
    """One line from the ESP32: a reply (kind "OK" or "ERR") or an event (kind "EV")."""

    kind: str
    name: str                      # the command's verb for a reply, the event's name for EV
    fields: dict = field(default_factory=dict, hash=False)

    @property
    def id(self):
        return self.fields.get("id")

    def get(self, key, default=None):
        return self.fields.get(key, default)

    def int(self, key):
        try:
            return int(self.fields[key])
        except (KeyError, ValueError):
            raise ProtocolError("%s %s has no whole number %s=" % (self.kind, self.name, key)) from None

    def float(self, key):
        try:
            return float(self.fields[key])
        except (KeyError, ValueError):
            raise ProtocolError("%s %s has no number %s=" % (self.kind, self.name, key)) from None

    def __str__(self):
        return " ".join([self.kind, self.name] + ["%s=%s" % kv for kv in self.fields.items()])


# where a message starts: not in the middle of a word or a key=value
_START = re.compile(r"(?<![A-Za-z0-9_=])(?:OK|ERR|EV) [A-Z?]")


def find_message(line):
    """The protocol message in a line read from the port, or None. After a reset the boot ROM
    prints at 115200 baud, which reads as a few bytes of garbage with no newline, so the EV BOOT
    that follows arrives glued to them (and to whatever was cut off by the reset). A clean line
    is taken whole; one with garbage in it, from its last message start on."""
    text = line.strip()
    if text.isascii() and text.isprintable() and _START.match(text):
        return text
    starts = list(_START.finditer(text))
    return text[starts[-1].start():] if starts else None


def parse(line):
    """One line from the ESP32 -> Message. Raises ProtocolError for anything that isn't protocol
    (the boot ROM's output after a reset, a garbled line), which the caller skips."""
    text = line.strip()
    kind, _, rest = text.partition(" ")
    if kind not in ("OK", "ERR", "EV"):
        raise ProtocolError("not a protocol line: %r" % text[:60])
    words = rest.split(" ")
    if not words[0]:
        raise ProtocolError("%s without a name" % kind)
    fields = {}
    for i, word in enumerate(words[1:], 1):
        if word.startswith("msg="):
            fields["msg"] = " ".join(words[i:])[4:]   # free text, always last
            break
        key, eq, value = word.partition("=")
        if eq:
            fields[key] = value
    return Message(kind, words[0].upper(), fields)
