#!/usr/bin/env python3
"""Run the robot simulation and verify generated autos round-trip through JSONHandler."""

from __future__ import annotations

import os
import queue
import signal
import http.client
import json
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_FILE = ROOT / "src" / "main" / "deploy" / "constants" / "config.json"
SIM_BASE_URL = "http://localhost:8088"
STARTUP_TIMEOUT_SECONDS = 90
PUBLISH_TIMEOUT_SECONDS = 30
SHUTDOWN_TIMEOUT_SECONDS = 10
WPILIB_JDK = Path(r"C:\Users\Public\wpilib\2026\jdk")


def detect_environment() -> str:
    try:
        config = json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
        env = config.get("selectedEnvironmentName") or config.get("defaultEnvironmentName", "comp")
        print(f"Detected environment: {env}", flush=True)
        return env
    except (FileNotFoundError, json.JSONDecodeError, KeyError) as exc:
        print(f"Warning: Could not read config.json ({exc}), defaulting to 'comp'", flush=True)
        return "comp"


def autos_url_for(env: str) -> str:
    return f"{SIM_BASE_URL}/{urllib.parse.quote(env, safe='')}/autos"


def subprocess_env() -> dict[str, str]:
    env = dict(os.environ)
    if os.name == "nt" and not env.get("JAVA_HOME") and (WPILIB_JDK / "bin" / "java.exe").exists():
        env["JAVA_HOME"] = str(WPILIB_JDK)
        env["PATH"] = str(WPILIB_JDK / "bin") + os.pathsep + env.get("PATH", "")
    return env


def gradlew() -> str:
    return str(ROOT / ("gradlew.bat" if os.name == "nt" else "gradlew"))


def reader_thread(stream, lines: "queue.Queue[str]") -> None:
    try:
        for line in stream:
            lines.put(line.rstrip())
    finally:
        stream.close()


def route_responds(autos_url: str) -> bool:
    try:
        with urllib.request.urlopen(autos_url, timeout=1) as response:
            return response.status == 200
    except (
        TimeoutError,
        ConnectionError,
        OSError,
        http.client.HTTPException,
        urllib.error.URLError,
        urllib.error.HTTPError,
    ):
        return False


def stop_process(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return

    if hasattr(os, "killpg"):
        os.killpg(process.pid, signal.SIGINT)
    elif os.name == "nt" and hasattr(signal, "CTRL_BREAK_EVENT"):
        try:
            process.send_signal(signal.CTRL_BREAK_EVENT)
        except (OSError, ValueError):
            process.terminate()
    else:
        process.send_signal(signal.SIGINT)

    try:
        process.wait(timeout=SHUTDOWN_TIMEOUT_SECONDS)
        return
    except subprocess.TimeoutExpired:
        pass

    if hasattr(os, "killpg"):
        os.killpg(process.pid, signal.SIGTERM)
    else:
        process.terminate()

    try:
        process.wait(timeout=SHUTDOWN_TIMEOUT_SECONDS)
        return
    except subprocess.TimeoutExpired:
        pass

    if hasattr(os, "killpg"):
        os.killpg(process.pid, signal.SIGKILL)
    else:
        process.kill()
    process.wait()


def wait_for_autos_route(
    process: subprocess.Popen[str], lines: "queue.Queue[str]", autos_url: str
) -> None:
    deadline = time.monotonic() + STARTUP_TIMEOUT_SECONDS

    while time.monotonic() < deadline:
        while True:
            try:
                line = lines.get_nowait()
            except queue.Empty:
                break

            print(line, flush=True)

        if process.poll() is not None:
            raise RuntimeError(f"simulateJava exited early with status {process.returncode}")

        if route_responds(autos_url):
            print("Autos tuning route is ready.", flush=True)
            return

        time.sleep(0.1)

    raise TimeoutError(f"Timed out waiting for {autos_url}")


def run_build_autos(env: str) -> None:
    result = subprocess.run(
        [sys.executable, "auto_generator/build_autos.py", "--sim", "--no-file", "--env", env],
        cwd=ROOT,
        env=subprocess_env(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=PUBLISH_TIMEOUT_SECONDS,
        check=False,
    )

    if result.returncode != 0:
        print(result.stdout, end="")
        raise RuntimeError(f"build_autos.py failed with status {result.returncode}")

    for line in result.stdout.splitlines():
        if (
            "Publishing to " in line
            or "PUT " in line
            or "POST " in line
            or "Round-trip structural check succeeded." in line
            or "Publish to " in line
        ):
            print(line, flush=True)


def bootstrap_generator(env: str) -> None:
    """Compile robot classes + dump the classpath for the Java auto generator.

    Done before launching simulateJava so the generator never has to invoke gradle
    concurrently with the running simulation. build_autos.py --no-file --bootstrap
    runs the full generator pipeline once (gradle compile + classpath dump + javac),
    discarding the output.
    """
    print("Bootstrapping Java auto generator (compile + classpath dump)...", flush=True)
    result = subprocess.run(
        [
            sys.executable,
            "auto_generator/build_autos.py",
            "--no-file",
            "--bootstrap",
            "--env",
            env,
        ],
        cwd=ROOT,
        env=subprocess_env(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=PUBLISH_TIMEOUT_SECONDS * 4,
        check=False,
    )
    if result.returncode != 0:
        print(result.stdout, end="")
        raise RuntimeError(f"generator bootstrap failed with status {result.returncode}")


def main() -> int:
    env = detect_environment()
    autos_url = autos_url_for(env)
    bootstrap_generator(env)

    process_options = {}
    if os.name == "nt":
        process_options["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
    else:
        process_options["start_new_session"] = True

    lines: "queue.Queue[str]" = queue.Queue()
    process = subprocess.Popen(
        [gradlew(), "--console=plain", "simulateJava"],
        cwd=ROOT,
        env=subprocess_env(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
        **process_options,
    )

    assert process.stdout is not None
    reader = threading.Thread(target=reader_thread, args=(process.stdout, lines), daemon=True)
    reader.start()

    try:
        wait_for_autos_route(process, lines, autos_url)
        run_build_autos(env)
        print("Auto JSON round-trip check passed.", flush=True)
        return 0
    finally:
        stop_process(process)
        reader.join(timeout=2)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
