#!/usr/bin/env python3
"""
build_autos.py

Discovers and imports all auto-definition modules in src/, runs them to
register autos with auto_lib, then writes a single Autos.json into
src/main/deploy/constants/comp/Autos.json.

Helper modules (auto_lib, shorthands, constants, field_locations, auto_action, units)
are excluded from auto-discovery.

Usage (from auto_generator/):
    python build_autos.py
"""

from __future__ import annotations

import importlib
import json
import os
import sys
from pathlib import Path

# Ensure the auto_generator directory is on the path so `src` is importable
_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))

from src import auto_lib

# Helper modules that should not be treated as auto files
EXCLUDED_MODULES = {
    "__init__",
    "auto_lib",
    "auto_action",
    "units",
    "shorthands",
    "constants",
    "field_locations",
}

OUTPUT_FILE = _SCRIPT_DIR.parent / "src" / "main" / "deploy" / "constants" / "comp" / "Autos.json"


def discover_auto_modules() -> list[str]:
    """Find all Python modules in src/ that are auto definitions (not helpers)."""
    src_dir = _SCRIPT_DIR / "src"
    modules = []
    for f in sorted(src_dir.iterdir()):
        if f.suffix == ".py" and f.stem not in EXCLUDED_MODULES:
            modules.append(f"src.{f.stem}")
    return modules


def main() -> None:
    # Step 1: Discover auto modules
    auto_modules = discover_auto_modules()

    if not auto_modules:
        print("No auto modules found in src/")
        return

    print(f"Found {len(auto_modules)} auto module(s): {[m.split('.')[-1] for m in auto_modules]}")

    # Step 2: Import each auto module so it registers its autos with auto_lib
    for module_name in auto_modules:
        print(f"  Importing {module_name}...")
        importlib.import_module(module_name)

    # Step 3: Write Autos.json
    autos = auto_lib.get_autos()

    if not autos:
        print("Warning: No autos were registered.")
    else:
        autos_obj = {}
        for name, action in autos.items():
            autos_obj[name] = auto_lib._serialize_value(action)

        output = {"autos": autos_obj}
        content = json.dumps(output, indent=4)

        OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT_FILE.write_text(content, encoding="utf-8")
        print(f"Written {len(autos)} auto(s) to: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
