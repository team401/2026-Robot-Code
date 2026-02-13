#!/usr/bin/env python3
"""Check that JSON constant files only contain keys matching Java class fields."""

import json
import os
import re
import subprocess
import sys

# Mapping from JSON filename to fully-qualified Java class name.
# Derived from JsonConstants.java lines 40-62.
JSON_TO_CLASS = {
    "RobotInfo.json": "frc.robot.constants.RobotInfo",
    "AprilTagConstants.json": "frc.robot.constants.AprilTagConstants",
    "FeatureFlags.json": "frc.robot.constants.FeatureFlags",
    "CANBusAssignment.json": "frc.robot.constants.CANBusAssignment",
    "DriveConstants.json": "frc.robot.constants.drive.DriveConstants",
    "VisionConstants.json": "frc.robot.constants.VisionConstants",
    "PhysicalDriveConstants.json": "frc.robot.constants.drive.PhysicalDriveConstants",
    "OperatorConstants.json": "frc.robot.constants.OperatorConstants",
    "HopperConstants.json": "frc.robot.constants.HopperConstants",
    "IndexerConstants.json": "frc.robot.constants.IndexerConstants",
    "TurretConstants.json": "frc.robot.constants.TurretConstants",
    "IntakeConstants.json": "frc.robot.constants.IntakeConstants",
    "ShooterConstants.json": "frc.robot.constants.ShooterConstants",
    "HoodConstants.json": "frc.robot.constants.HoodConstants",
    "ShotMaps.json": "frc.robot.constants.ShotMaps",
    "RedFieldLocations.json": "frc.robot.constants.FieldLocationInstance",
    "BlueFieldLocations.json": "frc.robot.constants.FieldLocationInstance",
}

SKIP_FILES = {"config.json", "controllers-xbox.json"}

BUILD_CLASSES_DIR = "build/classes/java/main"

# Pattern matching instance variable declarations from javap -p output, e.g.:
#   public double kP;
#   private final int count;
FIELD_PATTERN = re.compile(
    r"^\s+(?:public|protected|private)\s+(?:static\s+)?(?:final\s+)?\S+\s+(\w+);$"
)


def get_class_fields(class_name):
    """Get instance field names from a compiled .class file using javap."""
    result = subprocess.run(
        ["javap", "-p", "-cp", BUILD_CLASSES_DIR, class_name],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"WARNING: javap failed for {class_name}: {result.stderr.strip()}", file=sys.stderr)
        return None

    fields = set()
    for line in result.stdout.splitlines():
        m = FIELD_PATTERN.match(line)
        if m:
            fields.add(m.group(1))
    return fields


def get_json_keys(json_path):
    """Get top-level keys from a JSON file."""
    with open(json_path) as f:
        data = json.load(f)
    return set(data.keys())


def main():
    constants_dir = "src/main/deploy/constants"
    errors = []

    for env_name in sorted(os.listdir(constants_dir)):
        env_dir = os.path.join(constants_dir, env_name)
        if not os.path.isdir(env_dir):
            continue

        for json_file in sorted(os.listdir(env_dir)):
            if not json_file.endswith(".json") or json_file in SKIP_FILES:
                continue

            json_path = os.path.join(env_dir, json_file)

            class_name = JSON_TO_CLASS.get(json_file)
            if class_name is None:
                errors.append(f"ERROR: {json_path} has no class mapping")
                continue

            fields = get_class_fields(class_name)
            if fields is None:
                continue

            json_keys = get_json_keys(json_path)
            extra_keys = json_keys - fields

            for key in sorted(extra_keys):
                errors.append(f"ERROR: {json_path} has key '{key}' not found in {class_name}")

    for error in errors:
        print(error)

    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
