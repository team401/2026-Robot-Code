#!/usr/bin/env python3
"""Run the robot simulation and verify generated autos round-trip through JSONHandler."""

from __future__ import annotations

import os
import queue
import signal
import http.client
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
AUTOS_URL = "http://localhost:8088/comp/autos"
STARTUP_TIMEOUT_SECONDS = 90
PUBLISH_TIMEOUT_SECONDS = 30
SHUTDOWN_TIMEOUT_SECONDS = 10


def reader_thread(stream, lines: "queue.Queue[str]") -> None:
    try:
        for line in stream:
            lines.put(line.rstrip())
    finally:
        stream.close()


def route_responds() -> bool:
    try:
        with urllib.request.urlopen(AUTOS_URL, timeout=1) as response:
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


def wait_for_autos_route(process: subprocess.Popen[str], lines: "queue.Queue[str]") -> None:
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

        if route_responds():
            print("Autos tuning route is ready.", flush=True)
            return

        time.sleep(0.1)

    raise TimeoutError(f"Timed out waiting for {AUTOS_URL}")


def run_build_autos() -> None:
    result = subprocess.run(
        [sys.executable, "auto_generator/build_autos.py", "--sim", "--no-file"],
        cwd=ROOT,
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


def bootstrap_generator() -> None:
    """Compile robot classes + dump the classpath for the Java auto generator.

    Done before launching simulateJava so the generator never has to invoke gradle
    concurrently with the running simulation. build_autos.py --no-file --bootstrap
    runs the full generator pipeline once (gradle compile + classpath dump + javac),
    discarding the output.
    """
    print("Bootstrapping Java auto generator (compile + classpath dump)...", flush=True)
    result = subprocess.run(
        [sys.executable, "auto_generator/build_autos.py", "--no-file", "--bootstrap"],
        cwd=ROOT,
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
    bootstrap_generator()

    lines: "queue.Queue[str]" = queue.Queue()
    process = subprocess.Popen(
        ["./gradlew", "--console=plain", "simulateJava"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        bufsize=1,
    )

    assert process.stdout is not None
    reader = threading.Thread(target=reader_thread, args=(process.stdout, lines), daemon=True)
    reader.start()

    try:
        wait_for_autos_route(process, lines)
        run_build_autos()
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
