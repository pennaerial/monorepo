#!/usr/bin/env python3
"""Dump the live mode registry from vehicle_common as JSON.

Not a server -- invoked on demand (e.g. `python3 schema_export.py > schema.json`)
whenever the TypeScript apps need an up to date copy of the mission-mode schema
that drives the mission-graph/fleet editors. Requires a sourced ROS2 + colcon
environment where `vehicle_common` (and the `uav`/`payload` packages whose
modes it discovers) are importable.

Uses `vehicle_common.mode_loader.ModeRegistry.get()`, which does live
introspection of `@register_mode`-decorated classes -- not a static
`mode_registry.json` file. That file is referenced by the (now-deleted) old
integration backend, but the build step that would generate it is disabled
(see the commented-out `BuildUAVModeRegistry` step in `uav/setup.py`), and no
such file exists anywhere in this repo. Calling `ModeRegistry.get()` directly
is the mechanism that actually works today.

Phase 0 scope: dumps the raw mode registry only. Fleet/mission document JSON
schema (from `vehicle_common.runtime.fleet_spec`/`mission_spec`) and the
richer per-field shaping the old `schema.py` did (choices, defaults,
descriptions) lands in Phase 4, once the mission/fleet YAML editors are
actually being ported.
"""

from __future__ import annotations

import json
import sys


def main() -> int:
    try:
        from vehicle_common.mode_loader import ModeRegistry
    except ImportError as exc:
        print(
            f"Could not import vehicle_common.mode_loader: {exc}\n"
            "Run this from a sourced ROS2 + colcon environment where "
            "vehicle_common is built and on PYTHONPATH.",
            file=sys.stderr,
        )
        return 1

    registry = ModeRegistry.get()
    print(json.dumps(json.loads(registry.model_dump_json()), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
