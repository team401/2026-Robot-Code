#!/usr/bin/env bash
#
# Fast Java auto generator: compiles the auto-definition sources against the
# already-compiled robot classes and runs them, emitting Autos.json to stdout.
#
# This deliberately bypasses gradle on the hot path. A warm gradle daemon still
# spends ~4s configuring an up-to-date compileJava task on this project; a bare
# javac + java cycle is well under that. Gradle is only invoked for the one-time
# bootstrap (compile robot classes + dump the classpath), or when --bootstrap is
# passed (e.g. after editing robot/builder classes or changing dependencies).
#
# Usage:
#   generate.sh [ENVIRONMENT]            # print Autos.json for ENVIRONMENT (default: comp)
#   generate.sh --bootstrap [ENVIRONMENT] # force recompile of robot classes + classpath first
#
# All diagnostics go to stderr so stdout is clean JSON.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

CP_FILE="build/autogen-classpath.txt"
CLASSES="build/classes/java/main"
GEN_SRC="auto_generator_java"
GEN_OUT="build/autogen-out"
INIT_SCRIPT="$GEN_SRC/dump-classpath.gradle"

# Faster JVM startup for the short-lived javac/java processes.
JVM_FAST=(-XX:+UseSerialGC -XX:TieredStopAtLevel=1 -XX:-UsePerfData)

bootstrap=0
if [[ "${1:-}" == "--bootstrap" ]]; then
  bootstrap=1
  shift
fi
ENVIRONMENT="${1:-comp}"

if [[ "$bootstrap" == 1 || ! -f "$CP_FILE" || ! -d "$CLASSES" ]]; then
  echo "[generate] bootstrapping: gradle compileJava + classpath dump..." >&2
  ./gradlew --init-script "$INIT_SCRIPT" compileJava dumpAutoGenClasspath -q >&2
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
exec java "${JVM_FAST[@]}" -cp "$CP:$GEN_OUT" frc.robot.autogen.AutosGen "$ENVIRONMENT"
