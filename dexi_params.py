"""Load DEXI flight-controller params from the shared source of truth.

The params are NOT defined in this repo. They live in
DroneBlocks/dexi-fc-params -> src/dexi-3.json, vendored here at
params/dexi-3.json so bench provisioning works with no network.

    ./fetch-params.sh          # refresh the vendored copy from GitHub
    python3 -c "import dexi_params; print(len(dexi_params.load_profile('developer-kit')))"

Why a shared JSON: the same param set has four consumers in three languages —
this tool (Python), the px4-web-configurator (TypeScript), the QGC .params
downloads, and the PX4 airframe. A copy in each is how they silently diverge.
They already had: this script carried 36 params while the configurator wrote 47,
so a batch-flashed drone and a browser-provisioned one were different aircraft.
"""

from __future__ import annotations

import json
import pathlib
from typing import List, Tuple

# (name, value, kind) — the shape the provisioning scripts already use.
ParamOp = Tuple[str, float, str]

_HERE = pathlib.Path(__file__).parent
_SOURCE = _HERE / "params" / "dexi-3.json"


def _doc() -> dict:
    if not _SOURCE.exists():
        raise FileNotFoundError(
            f"{_SOURCE} is missing — run ./fetch-params.sh to pull it from dexi-fc-params"
        )
    return json.loads(_SOURCE.read_text())


def source_info() -> str:
    """One-line provenance, for scripts to print before they write anything."""
    d = _doc()
    return f"{d['name']} params v{d['schemaVersion']} (source: dexi-fc-params/src/dexi-3.json)"


def profile_keys() -> List[str]:
    return [p["key"] for p in _doc()["profiles"]]


def load_blocks(names: List[str]) -> List[ParamOp]:
    """Params for a list of block names, in order. Later wins on a repeat."""
    d = _doc()
    out: dict[str, ParamOp] = {}
    for name in names:
        block = d["blocks"].get(name)
        if block is None:
            raise KeyError(f'unknown param block "{name}" in {_SOURCE}')
        for p in block["params"]:
            out[p["name"]] = (p["name"], p["value"], p["kind"])
    return list(out.values())


def load_profile(key: str) -> List[ParamOp]:
    """Every param a named profile writes, in write order.

    Mirrors profileParams() in the configurator and tools/gen-params.mjs in
    dexi-fc-params — same JSON, same composition, same result.
    """
    d = _doc()
    prof = next((p for p in d["profiles"] if p["key"] == key), None)
    if prof is None:
        raise KeyError(f'unknown profile "{key}" — have: {", ".join(profile_keys())}')
    return load_blocks(prof["blocks"])


def notes() -> dict:
    """name -> note, for scripts that want to explain what they're writing."""
    d = _doc()
    return {
        p["name"]: p["note"]
        for b in d["blocks"].values()
        for p in b["params"]
        if p.get("note")
    }
