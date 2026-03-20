#!/usr/bin/env python3
"""
build_autos.py

Discovers and imports all auto-definition modules in src/, runs them to
register autos with auto_lib, then writes a single Autos.json and/or
publishes to the robot's tuning server.

Helper modules (auto_lib, shorthands, constants, field_locations, auto_action, units)
are excluded from auto-discovery.

Usage (from auto_generator/):
    python build_autos.py              # write Autos.json to deploy dir
    python build_autos.py --sim        # also publish to sim (localhost:8088)
    python build_autos.py --robot      # also publish to robot (10.4.1.2:8088)
    python build_autos.py --url HOST   # also publish to a custom address
    python build_autos.py --no-file    # skip writing the local file
"""

from __future__ import annotations

import argparse
import importlib
import json
import sys
import urllib.request
import urllib.error
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
    "routines",
}

OUTPUT_DIR = _SCRIPT_DIR.parent / "src" / "main" / "deploy" / "constants"
CONFIG_FILE = OUTPUT_DIR / "config.json"

SIM_URL = "http://localhost:8088"
ROBOT_URL = "http://10.4.1.2:8088"
ENDPOINT = "autos"


def detect_environment() -> str:
    """Read the selected environment from deploy/constants/config.json."""
    try:
        config = json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
        env = config.get("selectedEnvironmentName") or config.get("defaultEnvironmentName", "comp")
        print(f"Detected environment: {env}")
        return env
    except (FileNotFoundError, json.JSONDecodeError, KeyError) as e:
        print(f"Warning: Could not read config.json ({e}), defaulting to 'comp'")
        return "comp"


def discover_auto_modules() -> list[str]:
    """Find all Python modules in src/ that are auto definitions (not helpers)."""
    src_dir = _SCRIPT_DIR / "src"
    modules = []
    for f in sorted(src_dir.iterdir()):
        if f.suffix == ".py" and f.stem not in EXCLUDED_MODULES:
            modules.append(f"src.{f.stem}")
    return modules


def publish(base_url: str, env: str, content: str) -> None:
    """PUT the autos JSON to the tuning server, then POST to activate."""
    url = f"{base_url}/{env}/{ENDPOINT}"
    data = content.encode("utf-8")

    # PUT — upload the new data
    req = urllib.request.Request(url, data=data, method="PUT")
    req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            body = resp.read().decode("utf-8", errors="replace")
            print(f"  PUT {url} -> {resp.status}")
            if body:
                print(f"    Response: {body}")
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace") if e.fp else ""
        print(f"  PUT {url} failed: HTTP {e.code} {e.reason}")
        if body:
            print(f"    Response: {body}")
        return
    except urllib.error.URLError as e:
        print(f"  PUT {url} failed: {e.reason}")
        return
    except TimeoutError:
        print(f"  PUT {url} failed: Connection timed out")
        return

    # POST — tell the robot to reload autos
    req = urllib.request.Request(url, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            body = resp.read().decode("utf-8", errors="replace")
            print(f"  POST {url} -> {resp.status}")
            if body:
                print(f"    Response: {body}")
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace") if e.fp else ""
        print(f"  POST {url} failed: HTTP {e.code} {e.reason}")
        if body:
            print(f"    Response: {body}")
    except urllib.error.URLError as e:
        print(f"  POST {url} failed: {e.reason}")
    except TimeoutError:
        print(f"  POST {url} failed: Connection timed out")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build and publish autos")
    parser.add_argument(
        "--sim", action="store_true", help="Publish to sim (localhost:8088)"
    )
    parser.add_argument(
        "--robot", action="store_true", help="Publish to robot (10.4.1.2:8088)"
    )
    parser.add_argument(
        "--url", type=str, default=None, help="Publish to a custom URL (e.g. http://host:8088)"
    )
    parser.add_argument(
        "--no-file", action="store_true", help="Skip writing the local Autos.json file"
    )
    parser.add_argument(
        "--env", type=str, default=None, help="Override environment (default: auto-detect from config.json)"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    env = args.env if args.env else detect_environment()

    # Step 1: Import routines first so @routine decorators register before
    #         any auto modules try to reference them via routines.<name>()
    print("  Importing src.routines...")
    importlib.import_module("src.routines")

    # Step 2: Discover and import auto modules
    auto_modules = discover_auto_modules()

    if not auto_modules:
        print("No auto modules found in src/")
        return

    print(f"Found {len(auto_modules)} auto module(s): {[m.split('.')[-1] for m in auto_modules]}")

    for module_name in auto_modules:
        print(f"  Importing {module_name}...")
        importlib.import_module(module_name)

    # Step 3: Serialize
    autos = auto_lib.get_autos()
    routine_map = auto_lib.get_routines()

    if not autos and not routine_map:
        print("Warning: No autos or routines were registered.")
        return

    autos_obj = {}
    for name, action in autos.items():
        autos_obj[name] = auto_lib._serialize_value(action)

    routines_obj = {}
    for name, action in routine_map.items():
        routines_obj[name] = auto_lib._serialize_value(action)

    output = {"autos": autos_obj, "routines": routines_obj}
    content = json.dumps(output, indent=4)

    # Step 4: Write local file (unless --no-file)
    if not args.no_file:
        output_file = OUTPUT_DIR / env / "Autos.json"
        output_file.parent.mkdir(parents=True, exist_ok=True)
        output_file.write_text(content, encoding="utf-8")
        print(f"Written {len(autos)} auto(s) and {len(routine_map)} routine(s) to: {output_file}")

    # Step 5: Publish to tuning server(s)
    targets: list[str] = []
    if args.sim:
        targets.append(SIM_URL)
    if args.robot:
        targets.append(ROBOT_URL)
    if args.url:
        targets.append(args.url.rstrip("/"))

    for target in targets:
        print(f"Publishing to {target}...")
        publish(target, env, content)


if __name__ == "__main__":
    main()
