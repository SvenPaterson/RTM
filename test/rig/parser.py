"""Wire-frame parser for the RTM CC↔XPB UDP transport.

The transport carries ASCII line frames in the form:

    KIND;K1=V1;K2=V2;...:CC\n

where `CC` is an XOR checksum (uppercase hex) of every byte before the
`:` separator. The training newline is optional in the on-wire datagram
(present in practice, see tools/udp_capture.py captures).

Strict per-kind dataclasses are provided for the high-cadence frames
the harness asserts on directly (`HB`, `STAT`). All other kinds parse
to a `GenericFrame` that preserves the raw kv-pairs verbatim — promote
them to strict types when a scenario starts asserting on them.

Public API:
    parse_frame(payload, *, t_ms=0.0, src_ip="") -> Frame
    Frame                    — Union of HbFrame, StatFrame, GenericFrame
    HbFrame, StatFrame, GenericFrame
    BadChecksumError, MalformedFrameError

The parser is host-only Python — no firmware coupling beyond the wire
format documented in `src/shared/RtmComms.cpp`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Union


class FrameError(ValueError):
    """Base class for parser failures."""


class BadChecksumError(FrameError):
    """Wire checksum does not match the recomputed XOR."""


class MalformedFrameError(FrameError):
    """Frame structure (kind/sep/kv) is unparseable."""


# --- Per-kind strict dataclasses ----------------------------------------

@dataclass(frozen=True)
class _BaseFrame:
    # Capture-time metadata. Defaults so subclasses can be constructed
    # in tests without a live capture.
    t_ms: float = 0.0
    src_ip: str = ""
    raw: str = ""


@dataclass(frozen=True)
class HbFrame(_BaseFrame):
    """`HB;SEQ=n;STATE=...;STEP=n;LOOP=i/N;SW_AGE=ms;E=0|1;E_CODE=hh;RPM=n`

    Emitted by ClearCore. Nominal cadence 250 ms.
    """
    kind: str = "HB"
    seq: int = 0
    state: str = ""
    step: int = 0
    loop_idx: int = 0
    loop_total: int = 0
    sw_age_ms: int = 0
    e_flag: bool = False
    e_code: str = ""
    rpm: int = 0


@dataclass(frozen=True)
class StatFrame(_BaseFrame):
    """`STAT;SEQ=n;OUT=ddd;SUMP=t;SEAL=t`

    Emitted by XPB. Nominal cadence 1000 ms.
    """
    kind: str = "STAT"
    seq: int = 0
    out: str = ""       # 3-char digital output snapshot, kept as string
    sump_c: int = 0     # sump temperature, °C
    seal_c: int = 0     # seal temperature, °C


@dataclass(frozen=True)
class GenericFrame(_BaseFrame):
    """Any other frame kind (NOTICE, CMD, ACK, PR_BEG/DAT/END, ...).

    `fields` preserves insertion order from the wire so tests can match
    on positional semantics if they really need to.
    """
    kind: str = ""
    fields: Mapping[str, str] = field(default_factory=dict)


Frame = Union[HbFrame, StatFrame, GenericFrame]


# --- Internals ----------------------------------------------------------

def _calc_xor(payload: str) -> int:
    """XOR of all bytes in `payload` (the part before ':'). Matches
    `RtmComms::calculateXOR` in src/shared/RtmComms.cpp."""
    x = 0
    for ch in payload.encode("ascii", errors="strict"):
        x ^= ch
    return x


def _strip_wire_envelope(line: str) -> tuple[str, int | None]:
    """Strip trailing CR/LF and split off the checksum suffix if present.

    Returns (payload, checksum_or_None). Checksum is the integer value
    of the 2-hex suffix after the final ':'. If no ':' is present the
    payload is returned as-is and checksum is None — some debug paths
    elide the checksum.
    """
    s = line.rstrip("\r\n")
    if not s:
        raise MalformedFrameError("empty frame")
    sep = s.rfind(":")
    if sep < 0:
        return s, None
    suffix = s[sep + 1 :]
    if len(suffix) != 2:
        # ':' inside a value but no checksum — treat whole string as payload.
        return s, None
    try:
        cs = int(suffix, 16)
    except ValueError as exc:
        raise MalformedFrameError(f"bad checksum suffix {suffix!r}") from exc
    return s[:sep], cs


def _parse_kv(payload: str) -> tuple[str, dict[str, str]]:
    """Split `KIND;K=V;K=V` into (kind, {k: v}).

    Bare tokens (no '=') after the kind are tolerated and stored under
    their own key with an empty value — currently unused by known frame
    kinds but defensive.
    """
    parts = payload.split(";")
    kind = parts[0]
    if not kind:
        raise MalformedFrameError(f"missing kind in {payload!r}")
    fields: dict[str, str] = {}
    for tok in parts[1:]:
        if not tok:
            continue
        eq = tok.find("=")
        if eq < 0:
            fields[tok] = ""
        else:
            fields[tok[:eq]] = tok[eq + 1 :]
    return kind, fields


def _to_int(fields: Mapping[str, str], key: str) -> int:
    """Strict int field lookup — raises MalformedFrameError on absence
    or non-integer. Bare `int(...)` would raise KeyError/ValueError;
    we want a single exception type for callers."""
    if key not in fields:
        raise MalformedFrameError(f"missing field {key!r}")
    v = fields[key]
    try:
        return int(v)
    except ValueError as exc:
        raise MalformedFrameError(f"field {key}={v!r} not int") from exc


def _parse_loop(v: str) -> tuple[int, int]:
    """`'3/10'` → (3, 10). Anything else → MalformedFrameError."""
    if "/" not in v:
        raise MalformedFrameError(f"LOOP={v!r} missing '/'")
    a, b = v.split("/", 1)
    try:
        return int(a), int(b)
    except ValueError as exc:
        raise MalformedFrameError(f"LOOP={v!r} not int/int") from exc


def _build_hb(fields: Mapping[str, str], meta: dict) -> HbFrame:
    loop_idx, loop_total = _parse_loop(fields.get("LOOP", "0/0"))
    return HbFrame(
        seq=_to_int(fields, "SEQ"),
        state=fields.get("STATE", ""),
        step=_to_int(fields, "STEP"),
        loop_idx=loop_idx,
        loop_total=loop_total,
        sw_age_ms=_to_int(fields, "SW_AGE"),
        e_flag=fields.get("E", "0") != "0",
        e_code=fields.get("E_CODE", ""),
        rpm=_to_int(fields, "RPM"),
        **meta,
    )


def _build_stat(fields: Mapping[str, str], meta: dict) -> StatFrame:
    return StatFrame(
        seq=_to_int(fields, "SEQ"),
        out=fields.get("OUT", ""),
        sump_c=_to_int(fields, "SUMP"),
        seal_c=_to_int(fields, "SEAL"),
        **meta,
    )


_STRICT_BUILDERS = {
    "HB": _build_hb,
    "STAT": _build_stat,
}


# --- Public API ---------------------------------------------------------

def parse_frame(
    line: str,
    *,
    t_ms: float = 0.0,
    src_ip: str = "",
    verify_checksum: bool = True,
) -> Frame:
    """Parse a single wire frame.

    `line` may include trailing CR/LF and the `:CC` checksum suffix —
    both are tolerated and stripped. Set `verify_checksum=False` to
    accept lines without recomputing XOR (useful when replaying logs
    that have been re-escaped).

    Raises `BadChecksumError` if the suffix is present and mismatches,
    or `MalformedFrameError` for structural problems.
    """
    payload, wire_cs = _strip_wire_envelope(line)
    if verify_checksum and wire_cs is not None:
        actual = _calc_xor(payload)
        if actual != wire_cs:
            raise BadChecksumError(
                f"checksum mismatch: wire=0x{wire_cs:02X} actual=0x{actual:02X} "
                f"payload={payload!r}"
            )
    kind, fields = _parse_kv(payload)
    meta = {"t_ms": t_ms, "src_ip": src_ip, "raw": line}
    builder = _STRICT_BUILDERS.get(kind)
    if builder is not None:
        return builder(fields, meta)
    return GenericFrame(kind=kind, fields=fields, **meta)
