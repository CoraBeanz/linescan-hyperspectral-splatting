"""The serial protocol between the Jetson and the scan-mirror ESP32.

Plain ASCII, one message per line, fields separated by spaces, so it can be typed and read in
any serial monitor. Positions are whole microsteps counted from where the hall sensor triggers
(step 0); times are the ESP32's own microsecond clock (esp_timer). The ESP32 never deals in
angles: scan_mirror_bridge turns steps into mirror angles with scan_mirror.yaml.

    host -> ESP32                       ESP32 -> host
    PING <seq>                          PONG <seq> <t_us>
    STATUS                              STATUS <homed 0|1> <busy 0|1> <step> <sweep_id> <t_us>
    HOME                                OK HOME, later HOMED <step> <t_us>
    GOTO <step>                         OK GOTO
    SWEEP <id> <start_step> <steps_per_line> <n_lines> <period_us>
                                        OK SWEEP <id>, then one LINE per line and a DONE:
                                        LINE <id> <index> <step> <t_us>
                                        DONE <id> <t_us>
    STOP                                OK STOP
                                        ERR <command> <reason ...>   when a command is refused
                                        # <text>                     log text, ignored

LINE's t_us is the moment the mirror settled on that line. Everything about the wire format
is in this file, so matching a change in the firmware is a change here only.
"""

from dataclasses import dataclass


class ProtocolError(ValueError):
    pass


# --- host -> ESP32 ----------------------------------------------------------------------

def ping(seq):
    return "PING %d\n" % seq


def status():
    return "STATUS\n"


def home():
    return "HOME\n"


def goto(step):
    return "GOTO %d\n" % step


def sweep(sweep_id, start_step, steps_per_line, n_lines, period_us):
    if n_lines < 1 or period_us < 1:
        raise ProtocolError("a sweep needs at least one line and a positive period")
    return "SWEEP %d %d %d %d %d\n" % (sweep_id, start_step, steps_per_line, n_lines, period_us)


def stop():
    return "STOP\n"


# --- ESP32 -> host ----------------------------------------------------------------------

@dataclass(frozen=True)
class Pong:
    seq: int
    t_us: int


@dataclass(frozen=True)
class Status:
    homed: bool
    busy: bool
    step: int
    sweep_id: int
    t_us: int


@dataclass(frozen=True)
class Ok:
    command: str
    args: tuple = ()


@dataclass(frozen=True)
class Err:
    command: str
    reason: str


@dataclass(frozen=True)
class Homed:
    step: int
    t_us: int


@dataclass(frozen=True)
class Line:
    sweep_id: int
    index: int
    step: int
    t_us: int


@dataclass(frozen=True)
class Done:
    sweep_id: int
    t_us: int


@dataclass(frozen=True)
class Log:
    text: str


def _ints(fields, n, what):
    if len(fields) != n:
        raise ProtocolError("%s needs %d fields, got %d" % (what, n, len(fields)))
    try:
        return [int(f) for f in fields]
    except ValueError as e:
        raise ProtocolError("%s: %s" % (what, e)) from None


def parse(line):
    """One line from the ESP32 (without or with its newline) -> one of the classes above.
    Raises ProtocolError for anything malformed, so the caller can log it and carry on."""
    text = line.strip()
    if not text:
        raise ProtocolError("empty line")
    if text.startswith("#"):
        return Log(text[1:].strip())
    word, *fields = text.split()
    if word == "PONG":
        return Pong(*_ints(fields, 2, word))
    if word == "STATUS":
        homed, busy, step, sweep_id, t_us = _ints(fields, 5, word)
        return Status(bool(homed), bool(busy), step, sweep_id, t_us)
    if word == "LINE":
        return Line(*_ints(fields, 4, word))
    if word == "DONE":
        return Done(*_ints(fields, 2, word))
    if word == "HOMED":
        return Homed(*_ints(fields, 2, word))
    if word == "OK":
        if not fields:
            raise ProtocolError("OK without a command")
        return Ok(fields[0], tuple(fields[1:]))
    if word == "ERR":
        if not fields:
            raise ProtocolError("ERR without a command")
        return Err(fields[0], " ".join(fields[1:]))
    raise ProtocolError("unknown message %r" % word)
