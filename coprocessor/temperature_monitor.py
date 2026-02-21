#!/usr/bin/env python3
"""Reads Linux thermal zone temperatures and publishes them to NetworkTables."""

import argparse
import glob
import signal
import sys
import time
from pathlib import Path

import ntcore


def read_thermal_zones():
    """Discover and read all thermal zones, returning list of (zone_name, type, temp_c)."""
    zones = []
    for zone_dir in sorted(glob.glob("/sys/class/thermal/thermal_zone*/")):
        zone_path = Path(zone_dir)
        zone_name = zone_path.name
        try:
            zone_type = (zone_path / "type").read_text().strip()
            temp_millideg = int((zone_path / "temp").read_text().strip())
            temp_c = temp_millideg / 1000.0
            zones.append((zone_name, zone_type, temp_c))
        except (OSError, ValueError):
            continue
    return zones


def main():
    parser = argparse.ArgumentParser(description="Publish thermal zone temps to NetworkTables")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--team", type=int, help="FRC team number (connects via mDNS)")
    group.add_argument("--server", type=str, help="NT server host (e.g. localhost)")
    args = parser.parse_args()

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("temperature-monitor")
    if args.team is not None:
        inst.setServerTeam(args.team)
    else:
        inst.setServer(args.server)

    # Track publishers keyed by zone_name
    publishers: dict[str, ntcore.DoublePublisher] = {}

    running = True

    def shutdown(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    table = inst.getTable("/Coprocessor/thermal")

    while running:
        for zone_name, zone_type, temp_c in read_thermal_zones():
            if zone_name not in publishers:
                topic_name = f"{zone_type}_Celsius"
                publishers[zone_name] = table.getDoubleTopic(topic_name).publish()

            publishers[zone_name].set(temp_c)

        time.sleep(1)

    for pub in publishers.values():
        pub.close()
    inst.stopClient()


if __name__ == "__main__":
    main()
