"""Natural-language mission interpreter.

Input      : free text such as "Inspect the north zone and prioritize uninspected areas. Complete
             within 15 minutes."
Processing : (1) optional LLM call that must return a small JSON object, or (2) a deterministic
             local rule-based parser (always available, used as the fallback and in tests).
             Either way the result is re-validated by `validate_structured`: zone ids are checked
             against the real zones, numbers are clamped, unknown keys are dropped.
Output     : a structured, validated mission request (the same fields as POST /missions).

SAFETY BOUNDARY: the interpreter can only produce a *mission request*. It has no access to drones,
routes or control commands. Planning, drone selection, safety validation and execution remain in
the deterministic backend. An LLM failure, timeout or hallucination degrades to the local parser
or to a rejected request — never to an unvalidated action.
"""
from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass, field
from typing import Any, Callable

from app.core.enums import Capability

WORD_NUMBERS = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6}
UNSUPPORTED_VERBS = ["deliver", "strike", "attack", "drop", "pick up", "pickup", "follow", "track person",
                     "photograph person", "weapon"]
URGENT = r"\b(urgent|urgently|critical|asap|immediately|emergency|top priority|highest priority)\b"
HIGH = r"\b(high priority|important|priority (?:4|four))\b"
LOW = r"\b(low priority|routine|whenever|no rush|when convenient)\b"
UNINSPECTED = (r"(uninspected|un-inspected|not (?:yet )?(?:been )?inspected|haven'?t been inspected|"
               r"not inspected|new areas|unvisited|unchecked)")
REINSPECT = r"\b(re-?inspect|again|full re-?scan|re-?scan|everything|entire area|all areas|from scratch)\b"
INSPECT_VERBS = r"\b(inspect|inspection|survey|scan|check|examine|patrol|assess|monitor)\b"


@dataclass
class Interpretation:
    valid: bool
    source: str                                   # "local" | "llm"
    structured: dict[str, Any] = field(default_factory=dict)
    understood: dict[str, Any] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {"valid": self.valid, "source": self.source, "structured": self.structured,
                "understood": self.understood, "warnings": self.warnings, "errors": self.errors}


@dataclass(frozen=True)
class ZoneInfo:
    zone_id: str
    name: str


# ----------------------------------------------------------------------------- validation
def validate_structured(raw: dict, zones: list[ZoneInfo], text: str = "") -> tuple[dict, list[str], list[str]]:
    """Whitelist/clamp an untrusted dict (from the LLM or the local parser).

    Returns (clean_dict, warnings, errors). `errors` non-empty => the request must be rejected.
    """
    warnings: list[str] = []
    errors: list[str] = []
    zone_ids = {z.zone_id for z in zones}
    zone_id = raw.get("zone_id")
    if zone_id not in zone_ids:
        errors.append(f"unknown or missing zone '{zone_id}'; available: {sorted(zone_ids)}")

    objective = str(raw.get("objective", "inspection")).lower()
    if objective != "inspection":
        errors.append(f"unsupported objective '{objective}' (only 'inspection' is supported)")
        objective = "inspection"

    def clamp_int(key: str, default: int, lo: int, hi: int) -> int:
        v = raw.get(key, default)
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            warnings.append(f"{key} was not a number; using {default}")
            return default
        iv = int(round(v))
        if iv < lo or iv > hi:
            warnings.append(f"{key}={iv} out of range, clamped to [{lo}, {hi}]")
        return max(lo, min(hi, iv))

    priority = clamp_int("priority", 3, 1, 5)
    num_drones = clamp_int("num_drones", 1, 1, 6)

    deadline = raw.get("deadline_s")
    if deadline is not None:
        if isinstance(deadline, bool) or not isinstance(deadline, (int, float)) or deadline <= 0:
            warnings.append("deadline ignored (must be a positive number of seconds)")
            deadline = None
        elif deadline > 86400:
            warnings.append("deadline clamped to 24 h")
            deadline = 86400.0
        else:
            deadline = float(deadline)

    caps_raw = raw.get("required_capabilities") or ["CAMERA"]
    caps: list[str] = []
    for c in caps_raw if isinstance(caps_raw, list) else [caps_raw]:
        try:
            caps.append(Capability(str(c).upper()).value)
        except ValueError:
            warnings.append(f"unknown capability '{c}' ignored")
    if not caps:
        caps = ["CAMERA"]

    zone_name = next((z.name for z in zones if z.zone_id == zone_id), zone_id)
    name = str(raw.get("name") or f"Inspect {zone_name}").strip()[:200] or f"Inspect {zone_name}"
    clean = {"name": name, "description": text[:2000], "objective": objective, "priority": priority,
             "zone_id": zone_id, "deadline_s": deadline, "required_capabilities": sorted(set(caps)),
             "num_drones": num_drones, "include_inspected": bool(raw.get("include_inspected", False))}
    return clean, warnings, errors


# ----------------------------------------------------------------------------- local parser
def _zone_aliases(z: ZoneInfo) -> list[str]:
    ids = z.zone_id.lower().replace("_", " ")
    out = {z.name.lower(), ids}
    for direction in ("north", "south", "east", "west"):
        if direction in ids:
            out.update({f"{direction} zone", f"{direction}ern zone", f"{direction}", f"{direction}ern"})
    m = re.match(r"zone ([a-z0-9]+)$", ids)
    if m:
        k = m.group(1)
        out.update({f"zone {k}", f"area {k}", f"sector {k}", f"zone-{k}"})
    return sorted(out, key=len, reverse=True)


def parse_duration_seconds(text: str) -> float | None:
    unit = r"(seconds?|secs?|minutes?|mins?|hours?|hrs?|s|m|h)"
    pats = [rf"(?:within|in|under|by|before|inside|less than|no more than|max(?:imum)? of)\s+(\d+(?:\.\d+)?)\s*{unit}\b",
            rf"(\d+(?:\.\d+)?)\s*[- ]?{unit}\s+(?:deadline|limit|window)",
            rf"\b(\d+(?:\.\d+)?)\s*{unit}\b"]
    for p in pats:
        m = re.search(p, text)
        if m:
            val, u = float(m.group(1)), m.group(2)
            if u.startswith("h"):
                return val * 3600
            if u.startswith("m"):
                return val * 60
            return val
    m = re.search(r"\ban hour\b", text)
    if m:
        return 3600.0
    return None


def interpret_local(text: str, zones: list[ZoneInfo]) -> Interpretation:
    t = " " + re.sub(r"\s+", " ", text.lower()) + " "
    warnings: list[str] = []
    raw: dict[str, Any] = {"objective": "inspection"}

    for v in UNSUPPORTED_VERBS:
        if re.search(rf"\b{re.escape(v)}", t):
            return Interpretation(False, "local", errors=[f"unsupported objective: '{v}' "
                                                          "(only inspection missions are supported)"])

    if not re.search(INSPECT_VERBS, t):
        warnings.append("no inspection verb found; assuming an inspection mission")

    # zone: pick the earliest mention
    best: tuple[int, ZoneInfo] | None = None
    for z in zones:
        for alias in _zone_aliases(z):
            m = re.search(rf"(?<![a-z0-9]){re.escape(alias)}(?![a-z0-9])", t)
            if m and (best is None or m.start() < best[0]):
                best = (m.start(), z)
    mentioned = {z.zone_id for z in zones for a in _zone_aliases(z)
                 if re.search(rf"(?<![a-z0-9]){re.escape(a)}(?![a-z0-9])", t)}
    if best:
        raw["zone_id"] = best[1].zone_id
        if len(mentioned) > 1:
            warnings.append(f"several zones mentioned ({sorted(mentioned)}); using {best[1].zone_id}")
    else:
        return Interpretation(False, "local", warnings=warnings,
                              errors=[f"could not identify a zone; available: {[z.name for z in zones]}"])

    if re.search(URGENT, t):
        raw["priority"] = 5
    elif re.search(HIGH, t):
        raw["priority"] = 4
    elif re.search(LOW, t):
        raw["priority"] = 2

    dur = parse_duration_seconds(t)
    if dur is not None:
        raw["deadline_s"] = dur

    m = re.search(r"\b(\d+|one|two|three|four|five|six)\s+drones?\b", t)
    if m:
        tok = m.group(1)
        raw["num_drones"] = WORD_NUMBERS.get(tok) or int(tok)

    caps = ["CAMERA"]
    if "thermal" in t or "heat" in t or "infrared" in t:
        caps.append("THERMAL")
    if "lidar" in t or "3d scan" in t:
        caps.append("LIDAR")
    raw["required_capabilities"] = caps

    reinspect = bool(re.search(REINSPECT, t))
    prioritise_uninspected = bool(re.search(UNINSPECTED, t))
    raw["include_inspected"] = reinspect and not prioritise_uninspected

    clean, w2, errors = validate_structured(raw, zones, text)
    understood = {"objective": "inspection", "zone": clean.get("zone_id"),
                  "priority": "uninspected first" if prioritise_uninspected or not reinspect else "re-inspect all",
                  "deadline": f"{clean['deadline_s']:.0f} s" if clean.get("deadline_s") else None,
                  "drones": clean.get("num_drones"), "capabilities": clean.get("required_capabilities")}
    return Interpretation(not errors, "local", clean if not errors else {}, understood,
                          warnings + w2, errors)


# ----------------------------------------------------------------------------- LLM (optional)
LLM_SYSTEM = (
    "You convert a drone-fleet operator's request into ONE JSON object and nothing else. "
    "Allowed keys: objective (always \"inspection\"), zone_id (one of the provided ids), priority "
    "(integer 1-5), deadline_s (seconds or null), num_drones (integer 1-6), required_capabilities "
    "(subset of CAMERA, THERMAL, LIDAR), include_inspected (true only if the operator wants areas "
    "re-inspected), name (short title). You never issue drone commands, routes or coordinates. "
    "If the request is not an inspection mission, return {\"error\": \"unsupported\"}.")

HttpPost = Callable[[str, dict, dict], dict]


def _default_http_post(url: str, headers: dict, payload: dict) -> dict:
    import httpx
    r = httpx.post(url, headers=headers, json=payload, timeout=10.0)
    r.raise_for_status()
    return r.json()


class LLMInterpreter:
    def __init__(self, api_key: str, model: str = "claude-haiku-5-5", http_post: HttpPost | None = None,
                 url: str = "https://api.anthropic.com/v1/messages") -> None:
        self.api_key, self.model, self.url = api_key, model, url
        self.http_post = http_post or _default_http_post

    def interpret(self, text: str, zones: list[ZoneInfo]) -> Interpretation:
        payload = {
            "model": self.model, "max_tokens": 400, "system": LLM_SYSTEM,
            "messages": [{"role": "user", "content": json.dumps(
                {"available_zones": [{"zone_id": z.zone_id, "name": z.name} for z in zones],
                 "request": text})}]}
        headers = {"x-api-key": self.api_key, "anthropic-version": "2023-06-01",
                   "content-type": "application/json"}
        data = self.http_post(self.url, headers, payload)
        content = "".join(b.get("text", "") for b in data.get("content", []) if b.get("type") == "text")
        m = re.search(r"\{.*\}", content, re.S)
        if not m:
            raise ValueError("LLM returned no JSON object")
        raw = json.loads(m.group(0))
        if not isinstance(raw, dict):
            raise ValueError("LLM JSON was not an object")
        if "error" in raw:
            return Interpretation(False, "llm", errors=[f"LLM: {raw['error']}"])
        clean, warnings, errors = validate_structured(raw, zones, text)
        understood = {"objective": clean.get("objective"), "zone": clean.get("zone_id"),
                      "deadline": clean.get("deadline_s"), "drones": clean.get("num_drones")}
        return Interpretation(not errors, "llm", clean if not errors else {}, understood, warnings, errors)


class MissionInterpreter:
    """Facade: try the LLM when configured, otherwise (or on any failure) use the local parser."""

    def __init__(self, llm: LLMInterpreter | None = None) -> None:
        self.llm = llm

    @classmethod
    def from_env(cls) -> "MissionInterpreter":
        key = os.environ.get("ANTHROPIC_API_KEY")
        if key and os.environ.get("DRONEFLEET_USE_LLM", "1") != "0":
            return cls(LLMInterpreter(key, os.environ.get("DRONEFLEET_LLM_MODEL", "claude-haiku-5-5")))
        return cls(None)

    def interpret(self, text: str, zones: list[ZoneInfo]) -> Interpretation:
        text = text.strip()
        if self.llm is not None:
            try:
                res = self.llm.interpret(text, zones)
                return res
            except Exception as e:                      # network, auth, malformed output ...
                local = interpret_local(text, zones)
                local.warnings.insert(0, f"LLM unavailable ({type(e).__name__}); used local parser")
                return local
        return interpret_local(text, zones)
