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

# Matches field declarations in javap -verbose output, e.g.:
#   public double kP;
#   private final int count;
# But NOT method declarations (which have parentheses in the descriptor).
FIELD_PATTERN = re.compile(
    r"^\s+(?:public|protected|private)\s+(static\s+)?(?:final\s+)?\S+\s+(\w+);$"
)

JSON_EXCLUDE_ANNOTATION = "coppercore.parameter_tools.json.annotations.JSONExclude"


def get_class_fields(class_name):
    """Get field names and excluded field names from a .class file using javap -verbose.

    Returns (all_fields, excluded_fields) where both are sets of field name strings,
    or (None, None) if javap fails.
    """
    result = subprocess.run(
        ["javap", "-p", "-verbose", "-cp", BUILD_CLASSES_DIR, class_name],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"WARNING: javap failed for {class_name}: {result.stderr.strip()}", file=sys.stderr)
        return None, None

    all_fields = set()
    excluded_fields = set()

    lines = result.stdout.splitlines()
    i = 0
    while i < len(lines):
        m = FIELD_PATTERN.match(lines[i])
        if m:
            is_static = m.group(1) is not None
            field_name = m.group(2)
            if is_static:
                i += 1
                continue
            all_fields.add(field_name)
            # Scan the indented block following this field for annotations
            j = i + 1
            while j < len(lines) and lines[j].startswith("    "):
                if JSON_EXCLUDE_ANNOTATION in lines[j]:
                    excluded_fields.add(field_name)
                j += 1
        i += 1

    return all_fields, excluded_fields


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

            all_fields, excluded_fields = get_class_fields(class_name)
            if all_fields is None:
                continue

            json_keys = get_json_keys(json_path)

            # JSON keys not in the class
            extra_keys = json_keys - all_fields
            for key in sorted(extra_keys):
                errors.append(f"ERROR: {json_path} has key '{key}' not found in {class_name}")

            # Class fields (non-excluded) not in JSON
            expected_fields = all_fields - excluded_fields
            missing_keys = expected_fields - json_keys
            for key in sorted(missing_keys):
                errors.append(f"ERROR: {json_path} is missing key '{key}' expected by {class_name}")

    for error in errors:
        print(error)

    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
