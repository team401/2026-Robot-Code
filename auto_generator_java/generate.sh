#!/usr/bin/env bash
#
# Fast Java auto generator: compiles the auto-definition sources against the
# already-compiled robot classes and runs them, emitting Autos.json to stdout.
#
# The generator runs as a real (if headless) robot JVM: the WPILib native
# libraries are put on the library path so the auto code has full, unrestricted
# access to the rest of the robot code (HAL, Filesystem, NetworkTables, etc.),
# exactly as it would on the robot.
#
# This deliberately bypasses gradle on the hot path. A warm gradle daemon still
# spends ~4s configuring an up-to-date compileJava task on this project; a bare
# javac + java cycle is well under that. Gradle is only invoked for the one-time
# bootstrap (compile robot classes, extract native libs, dump the classpath), or
# when --bootstrap is passed (e.g. after editing robot/builder code or changing
# dependencies).
#
# Usage:
#   generate.sh [ENVIRONMENT]             # print Autos.json for ENVIRONMENT (default: comp)
#   generate.sh --bootstrap [ENVIRONMENT] # force recompile + native extraction + classpath dump
#
# All diagnostics (and native HAL stdout chatter) go to stderr so stdout is clean JSON.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

CP_FILE="build/autogen-classpath.txt"
CLASSES="build/classes/java/main"
NATIVE_DIR="build/jni/release"
GEN_SRC="auto_generator_java"
GEN_OUT="build/autogen-out"
OUT_JSON="$GEN_OUT/Autos.json"
INIT_SCRIPT="$GEN_SRC/dump-classpath.gradle"

# Faster JVM startup for the short-lived javac/java processes.
JVM_FAST=(-XX:+UseSerialGC -XX:TieredStopAtLevel=1 -XX:-UsePerfData)

bootstrap=0
if [[ "${1:-}" == "--bootstrap" ]]; then
  bootstrap=1
  shift
fi
ENVIRONMENT="${1:-comp}"

if [[ "$bootstrap" == 1 || ! -f "$CP_FILE" || ! -d "$CLASSES" || ! -d "$NATIVE_DIR" ]]; then
  echo "[generate] bootstrapping: gradle compileJava + native extraction + classpath dump..." >&2
  ./gradlew --init-script "$INIT_SCRIPT" compileJava extractReleaseNative dumpAutoGenClasspath -q >&2
fi

CP="$(cat "$CP_FILE"):$CLASSES"
mkdir -p "$GEN_OUT"

# Skip compilation if no generator source is newer than the last build output.
needs_compile=1
marker="$GEN_OUT/.compiled"
if [[ -f "$marker" && -z "$(find "$GEN_SRC" -name '*.java' -newer "$marker" -print -quit)" ]]; then
  needs_compile=0
fi

if [[ "$needs_compile" == 1 ]]; then
  echo "[generate] compiling generator sources..." >&2
  # shellcheck disable=SC2046
  javac -J-XX:+UseSerialGC -J-XX:TieredStopAtLevel=1 -J-XX:-UsePerfData \
    -cp "$CP" -d "$GEN_OUT" $(find "$GEN_SRC" -name '*.java')
  touch "$marker"
else
  echo "[generate] generator sources unchanged; skipping compile" >&2
fi

echo "[generate] running generator for environment '$ENVIRONMENT'..." >&2
# Native JNI libs (and their transitive .so deps) must be resolvable both for the
# JVM's System.loadLibrary (java.library.path) and the dynamic linker (LD_LIBRARY_PATH).
export LD_LIBRARY_PATH="$ROOT/$NATIVE_DIR:${LD_LIBRARY_PATH:-}"
# Send the JVM's own stdout (HAL native chatter) to stderr; the generator writes the
# JSON to OUT_JSON, which we then emit on stdout so callers get clean JSON.
java "${JVM_FAST[@]}" -Djava.library.path="$ROOT/$NATIVE_DIR" \
  -cp "$CP:$GEN_OUT" frc.robot.autogen.AutosGen "$ENVIRONMENT" "$OUT_JSON" 1>&2
cat "$OUT_JSON"
