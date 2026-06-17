#!/usr/bin/env python3
"""
Cross-platform driver for the Java auto generator.

Compiles the auto-definition sources (src/main/java/frc/robot/autogen) against the
already-compiled robot classes and runs them as a (headless) robot JVM with the WPILib
native libraries on the path, so the auto code has full, unrestricted access to
the rest of the robot code (HAL, Filesystem, NetworkTables), exactly as on the
robot. Emits Autos.json to stdout.

This deliberately bypasses gradle on the hot path. A warm gradle daemon still
spends ~4s configuring an up-to-date compileJava task on this project; a bare
javac + java cycle is well under that. Gradle is only invoked for the one-time
bootstrap (compile robot classes, extract native libs, dump the classpath), or
when --bootstrap is passed (e.g. after editing robot/builder code or changing
dependencies).

Usage:
    python generate.py [ENVIRONMENT]              # print Autos.json (default env: comp)
    python generate.py --bootstrap [ENVIRONMENT]  # force recompile + native extraction + dump

All diagnostics (and native HAL stdout chatter) go to stderr so stdout is clean JSON.
"""
from __future__ import annotations

import os
import platform
import subprocess
import sys
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent  # auto_generator/
_ROOT = _THIS_DIR.parent

CP_FILE = _ROOT / "build" / "autogen-classpath.txt"
CLASSES = _ROOT / "build" / "classes" / "java" / "main"
NATIVE_DIR = _ROOT / "build" / "jni" / "release"
# Only the autogen package is fast-compiled on the hot path (the rest of the robot
# is already compiled into CLASSES by the bootstrap's gradle compileJava).
GEN_SRC = _ROOT / "src" / "main" / "java" / "frc" / "robot" / "autogen"
GEN_OUT = _ROOT / "build" / "autogen-out"
OUT_JSON = GEN_OUT / "Autos.json"
MARKER = GEN_OUT / ".compiled"
INIT_SCRIPT = _THIS_DIR / "dump-classpath.gradle"
MAIN_CLASS = "frc.robot.autogen.GenerateAutos"

_IS_WINDOWS = os.name == "nt"
JVM_FAST = ["-XX:+UseSerialGC", "-XX:TieredStopAtLevel=1", "-XX:-UsePerfData"]


def _log(message: str) -> None:
    print(f"[generate] {message}", file=sys.stderr, flush=True)


def _gradlew() -> str:
    return str(_ROOT / ("gradlew.bat" if _IS_WINDOWS else "gradlew"))


def _jdk_tool(name: str) -> str:
    """Resolve a JDK tool (java/javac), preferring JAVA_HOME (set by the WPILib env)."""
    java_home = os.environ.get("JAVA_HOME")
    if java_home:
        exe = name + (".exe" if _IS_WINDOWS else "")
        candidate = Path(java_home) / "bin" / exe
        if candidate.exists():
            return str(candidate)
    return name  # fall back to PATH


def _java_sources() -> list[str]:
    return [str(p) for p in sorted(GEN_SRC.rglob("*.java"))]


def _sources_changed() -> bool:
    if not MARKER.exists():
        return True
    marker_mtime = MARKER.stat().st_mtime
    return any(Path(src).stat().st_mtime > marker_mtime for src in _java_sources())


def _bootstrap() -> None:
    _log("bootstrapping: gradle compileJava + native extraction + classpath dump...")
    subprocess.run(
        [
            _gradlew(),
            "--init-script",
            str(INIT_SCRIPT),
            "compileJava",
            "extractReleaseNative",
            "dumpAutoGenClasspath",
            "-q",
        ],
        cwd=str(_ROOT),
        check=True,
        stdout=sys.stderr,  # keep gradle chatter out of stdout (which carries the JSON)
    )


def _native_loader_env() -> dict[str, str]:
    """Environment with the native lib dir prepended to the OS's dynamic-loader path."""
    env = dict(os.environ)
    var = {
        "Windows": "PATH",
        "Darwin": "DYLD_LIBRARY_PATH",
    }.get(platform.system(), "LD_LIBRARY_PATH")
    existing = env.get(var, "")
    env[var] = str(NATIVE_DIR) + (os.pathsep + existing if existing else "")
    return env


def generate(env: str = "comp", bootstrap: bool = False) -> str:
    """Build the autos and return the Autos.json content for `env`."""
    if bootstrap or not CP_FILE.exists() or not CLASSES.is_dir() or not NATIVE_DIR.is_dir():
        _bootstrap()

    classpath = CP_FILE.read_text(encoding="utf-8").strip() + os.pathsep + str(CLASSES)
    GEN_OUT.mkdir(parents=True, exist_ok=True)

    if _sources_changed():
        _log("compiling generator sources...")
        subprocess.run(
            [
                _jdk_tool("javac"),
                "-J-XX:+UseSerialGC",
                "-J-XX:TieredStopAtLevel=1",
                "-J-XX:-UsePerfData",
                "-cp",
                classpath,
                "-d",
                str(GEN_OUT),
                *_java_sources(),
            ],
            cwd=str(_ROOT),
            check=True,
        )
        MARKER.touch()
    else:
        _log("generator sources unchanged; skipping compile")

    # The generator selects the environment from the deploy constants/config.json (the same way
    # the robot does); `env` here is only what the caller intends to write to, for logging.
    _log(f"running generator (requested env '{env}'; environment comes from config.json)...")
    # GEN_OUT goes first so the freshly fast-compiled autogen classes take precedence
    # over the (possibly stale) copies the gradle bootstrap compiled into CLASSES.
    run_classpath = str(GEN_OUT) + os.pathsep + classpath
    # The generator writes JSON to OUT_JSON. Initializing HAL prints native chatter to
    # the JVM's stdout, so we route that to our stderr to keep it out of the JSON.
    subprocess.run(
        [
            _jdk_tool("java"),
            *JVM_FAST,
            f"-Djava.library.path={NATIVE_DIR}",
            "-cp",
            run_classpath,
            MAIN_CLASS,
            str(OUT_JSON),
        ],
        cwd=str(_ROOT),
        check=True,
        stdout=sys.stderr,
        env=_native_loader_env(),
    )
    return OUT_JSON.read_text(encoding="utf-8")


def main(argv: list[str]) -> int:
    bootstrap = False
    args = list(argv)
    if args and args[0] == "--bootstrap":
        bootstrap = True
        args = args[1:]
    env = args[0] if args else "comp"
    sys.stdout.write(generate(env, bootstrap))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except subprocess.CalledProcessError as exc:
        print(f"[generate] command failed ({exc.returncode}): {exc.cmd}", file=sys.stderr)
        raise SystemExit(exc.returncode)
