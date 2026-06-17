#!/usr/bin/env python3
"""
build_autos.py

Orchestrates auto generation and publishing. The autos themselves are authored
in Java (see auto_generator_java/) and serialized to Autos.json by a fast Java
generator that reuses the robot's own coppercore Gson configuration. This script
invokes that generator, then writes the local Autos.json and/or publishes to the
robot's tuning server (verifying a structural round-trip on PUT).

Usage (from the repo root or auto_generator/):
    python build_autos.py              # write Autos.json to deploy dir
    python build_autos.py --sim        # also publish to sim (localhost:8088)
    python build_autos.py --robot      # also publish to robot (10.4.1.2:8088)
    python build_autos.py --url HOST   # also publish to a custom address
    python build_autos.py --no-file    # skip writing the local file
    python build_autos.py --bootstrap  # recompile robot classes + classpath first
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import urllib.request
import urllib.error
from pathlib import Path
from typing import Any

_SCRIPT_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _SCRIPT_DIR.parent

# The autos are authored in Java and serialized to Autos.json by this generator,
# which reuses the robot's own coppercore Gson configuration. See auto_generator_java/.
GENERATOR = _REPO_ROOT / "auto_generator_java" / "generate.sh"

OUTPUT_DIR = _REPO_ROOT / "src" / "main" / "deploy" / "constants"
CONFIG_FILE = OUTPUT_DIR / "config.json"

SIM_URL = "http://localhost:8088"
ROBOT_URL = "http://10.4.1.2:8088"
ENDPOINT = "autos"


class PublishError(RuntimeError):
    pass


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


def generate_autos_json(env: str, bootstrap: bool = False) -> str:
    """Run the Java auto generator and return the Autos.json content for `env`.

    The generator prints clean JSON to stdout and progress/diagnostics to stderr.
    """
    cmd = [str(GENERATOR)]
    if bootstrap:
        cmd.append("--bootstrap")
    cmd.append(env)
    result = subprocess.run(
        cmd,
        cwd=str(_REPO_ROOT),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.stderr:
        print(result.stderr, file=sys.stderr, end="")
    if result.returncode != 0:
        raise SystemExit(f"Java auto generator failed with status {result.returncode}")
    return result.stdout


def _is_json_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _first_structural_difference(expected: Any, actual: Any, path: str = "$") -> str | None:
    if isinstance(expected, bool) or isinstance(actual, bool):
        if expected is actual:
            return None
        return f"{path}: expected {expected!r}, got {actual!r}"

    if _is_json_number(expected) and _is_json_number(actual):
        if expected == actual:
            return None
        return f"{path}: expected {expected!r}, got {actual!r}"

    if type(expected) is not type(actual):
        return f"{path}: expected {type(expected).__name__}, got {type(actual).__name__}"

    if isinstance(expected, dict):
        expected_keys = set(expected)
        actual_keys = set(actual)
        if expected_keys != actual_keys:
            missing = sorted(expected_keys - actual_keys)
            extra = sorted(actual_keys - expected_keys)
            if missing:
                return f"{path}: missing key {missing[0]!r}"
            return f"{path}: unexpected key {extra[0]!r}"
        for key in sorted(expected_keys):
            child_path = f"{path}.{key}" if key.isidentifier() else f"{path}[{key!r}]"
            difference = _first_structural_difference(expected[key], actual[key], child_path)
            if difference:
                return difference
        return None

    if isinstance(expected, list):
        if len(expected) != len(actual):
            return f"{path}: expected list length {len(expected)}, got {len(actual)}"
        for index, (expected_item, actual_item) in enumerate(zip(expected, actual)):
            difference = _first_structural_difference(expected_item, actual_item, f"{path}[{index}]")
            if difference:
                return difference
        return None

    if expected != actual:
        return f"{path}: expected {expected!r}, got {actual!r}"
    return None


def verify_round_trip(sent_content: str, returned_content: str) -> None:
    try:
        sent_json = json.loads(sent_content)
    except json.JSONDecodeError as e:
        raise PublishError(f"Generated autos JSON is invalid: {e}") from e

    try:
        returned_json = json.loads(returned_content)
    except json.JSONDecodeError as e:
        raise PublishError(f"Server returned invalid JSON from PUT: {e}") from e

    difference = _first_structural_difference(sent_json, returned_json)
    if difference:
        raise PublishError(f"Server round-trip JSON is not structurally equivalent: {difference}")


def publish(base_url: str, env: str, content: str) -> None:
    """PUT the autos JSON, verify the returned JSON, then POST to activate."""
    url = f"{base_url}/{env}/{ENDPOINT}"
    data = content.encode("utf-8")

    # PUT - upload the new data and validate the Java-side serialization.
    req = urllib.request.Request(url, data=data, method="PUT")
    req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            body = resp.read().decode("utf-8", errors="replace")
            print(f"  PUT {url} -> {resp.status}")
            verify_round_trip(content, body)
            print("    Round-trip structural check succeeded.")
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace") if e.fp else ""
        message = f"PUT {url} failed: HTTP {e.code} {e.reason}"
        if body:
            message += f"\n    Response: {body}"
        raise PublishError(message) from e
    except urllib.error.URLError as e:
        raise PublishError(f"PUT {url} failed: {e.reason}") from e
    except TimeoutError:
        raise PublishError(f"PUT {url} failed: Connection timed out")

    # POST - tell the robot to reload autos.
    req = urllib.request.Request(url, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            # Drain the response but don't echo it; on success the body is just the
            # full autos JSON, which is only noise. Failures are reported below.
            resp.read()
            print(f"  POST {url} -> {resp.status}")
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace") if e.fp else ""
        message = f"POST {url} failed: HTTP {e.code} {e.reason}"
        if body:
            message += f"\n    Response: {body}"
        raise PublishError(message) from e
    except urllib.error.URLError as e:
        raise PublishError(f"POST {url} failed: {e.reason}") from e
    except TimeoutError:
        raise PublishError(f"POST {url} failed: Connection timed out")


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
    parser.add_argument(
        "--bootstrap",
        action="store_true",
        help="Force the Java generator to recompile robot classes and re-dump the classpath "
        "(needed after changing robot/builder code or dependencies)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    env = args.env if args.env else detect_environment()

    # Generate Autos.json by running the Java auto generator (reuses the robot's
    # own coppercore Gson config, so the output round-trips by construction).
    content = generate_autos_json(env, bootstrap=args.bootstrap)

    # Write local file (unless --no-file)
    if not args.no_file:
        output_file = OUTPUT_DIR / env / "Autos.json"
        output_file.parent.mkdir(parents=True, exist_ok=True)
        output_file.write_text(content, encoding="utf-8")
        print(f"Written autos to: {output_file}")

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
        try:
            publish(target, env, content)
            print(f"Publish to {target} succeeded.")
        except PublishError as e:
            print(f"Publish to {target} failed: {e}", file=sys.stderr)
            raise SystemExit(1) from e


if __name__ == "__main__":
    main()
