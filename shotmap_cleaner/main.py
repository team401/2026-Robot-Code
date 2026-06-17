import json
import pandas as pd
import numpy as np
from InquirerPy import inquirer
from InquirerPy.base.control import Choice
from typing import TypedDict, Literal
import matplotlib.pyplot as plt


class UnitedValue(TypedDict):
    value: float
    unit: Literal["Degree"] | Literal["Second"] | Literal["Meter"]


class MapDataPoint(TypedDict):
    distance: UnitedValue
    shooterRPM: float
    hoodAngle: UnitedValue
    flightTime: UnitedValue


class ShotMaps(TypedDict):
    hubDataPoints: list[MapDataPoint]
    passDataPoints: list[MapDataPoint]
    mechanismCompensationDelay: UnitedValue
    rpmCompensation: UnitedValue


class ShotMapDFDict(TypedDict):
    distance: list[float]
    rpm: list[float]
    pitchDegrees: list[float]
    flightTimeSeconds: list[float]


def number(n: str) -> int | float:
    try:
        if float(int(n)) != float(n):
            raise Exception()
        return int(n)
    except:
        return float(n)


def convert_shotmaps_to_frame(shot_map_data: ShotMaps) -> ShotMapDFDict:
    output: ShotMapDFDict = {
        "distance": [],
        "rpm": [],
        "pitchDegrees": [],
        "flightTimeSeconds": [],
    }

    for data_point in shot_map_data["hubDataPoints"]:
        output["distance"].append(data_point["distance"]["value"])
        output["rpm"].append(data_point["shooterRPM"])
        output["pitchDegrees"].append(data_point["hoodAngle"]["value"])
        output["flightTimeSeconds"].append(data_point["flightTime"]["value"])

    return output


def convert_frame_to_shotmaps(frame: pd.DataFrame) -> ShotMaps:
    with open(
        "../2026-Robot-Code/src/main/deploy/constants/comp/ShotMaps.json", "r"
    ) as f:
        shot_map_data = json.load(f)

    shot_map_data["hubDataPoints"] = []

    for distance in frame.index:
        shot_map_data["hubDataPoints"].append(
            {
                "distance": {
                    "value": number(distance),
                    "unit": "Meter",
                },
                "shooterRPM": number(frame["rpm"][distance]),
                "hoodAngle": {
                    "value": number(frame["pitchDegrees"][distance]),
                    "unit": "Degree",
                },
                "flightTime": {
                    "value": number(frame["flightTimeSeconds"][distance]),
                    "unit": "Second",
                },
            }
        )

    return shot_map_data


def filter_and_reindex(df, selected_distances):
    filtered_df = df[df["distance"].isin(selected_distances)]

    filtered_df = filtered_df.set_index("distance", drop=True)
    df = df.set_index("distance", drop=True)

    return df, filtered_df


def get_residuals(df, filtered_df, resolution=0.01):
    min_dist = df.index.min()
    max_dist = df.index.max()

    # Dense grid using linspace avoids np.arange floating-point drift
    dense_index = pd.Index(
        np.linspace(min_dist, max_dist, int((max_dist - min_dist) / resolution) + 1)
    )

    # Union of filtered points + dense grid, then interpolate
    combined_index = filtered_df.index.union(dense_index)
    interpolated = (
        filtered_df.reindex(combined_index)
        .sort_index()
        .interpolate(method="linear")
        .reindex(dense_index)
    )

    # Do the same for df to get the "ground truth" at high resolution
    full_combined_index = df.index.union(dense_index)
    full_interpolated = (
        df.reindex(full_combined_index)
        .sort_index()
        .interpolate(method="linear")
        .reindex(dense_index)
    )

    return interpolated - full_interpolated


def evaluate(df, filtered_df):
    residuals = get_residuals(df, filtered_df)

    # print("residuals:\n", residuals)
    # print(residuals.describe())
    residuals_abs = residuals.abs()
    print("residuals abs:")
    print(residuals_abs.describe())

    print("Points with large error:")
    print(residuals_abs[residuals_abs["rpm"] > 25.0])


def main():
    with open("../src/main/deploy/constants/comp/ShotMaps.json", "r") as f:
        shot_map_data = json.load(f)

    shot_map_df_dict = convert_shotmaps_to_frame(shot_map_data)

    selected_distances = []
    while True:
        df = pd.DataFrame(shot_map_df_dict).sort_values(by="distance")

        action_choices = [
            Choice(value="manual", name="Select points manually"),
            Choice(value="auto", name="Automatically filter points"),
        ]
        mode = inquirer.select(
            message="Select filtering method",
            choices=action_choices,
            vi_mode=True,
            long_instruction="(j/k to move down/up, enter to select)",
        ).execute()

        if mode == "manual":
            distance_choices = [
                Choice(value=d, name=f"{d} meters", enabled=d in selected_distances)
                for d in df["distance"]
            ]
            selected_distances = inquirer.checkbox(
                message="Select points to use",
                long_instruction="(j/k to move down/up, space to toggle 1, C-r to toggle all, enter to try this set)",
                choices=distance_choices,
                vi_mode=True,
                default=selected_distances,
            ).execute()
            df, filtered_df_to_write = filter_and_reindex(df, selected_distances)
        else:
            max_rpm_error = float(
                inquirer.number(
                    message="Max RPM error",
                    instruction="(RPM)",
                    vi_mode=True,
                    float_allowed=True,
                    min_allowed=0,
                    transformer=lambda x: x + " RPM",
                ).execute()
            )
            max_angle_error = float(
                inquirer.number(
                    message="Max hood angle error",
                    instruction="(degrees)",
                    vi_mode=True,
                    float_allowed=True,
                    min_allowed=0,
                    transformer=lambda x: x + " degrees",
                ).execute()
            )
            max_tof_error = float(
                inquirer.number(
                    message="Max time of flight error",
                    instruction="(seconds)",
                    vi_mode=True,
                    float_allowed=True,
                    min_allowed=0,
                    transformer=lambda x: x + " seconds",
                ).execute()
            )

            df = df.set_index("distance", drop=True)
            filtered_df = df.copy()

            while True:
                best_distance = None
                best_max_rpm_err = float("inf")
                best_max_angle_err = float("inf")
                best_max_tof_err = float("inf")

                for distance in filtered_df.index:
                    # Never remove endpoints — interpolation can't extrapolate
                    if (
                        distance == filtered_df.index.min()
                        or distance == filtered_df.index.max()
                    ):
                        continue

                    trial_filtered_df = filtered_df[filtered_df.index != distance]
                    residuals = get_residuals(df, trial_filtered_df).abs()

                    # If NaNs exist, the trial doesn't cover the full range — skip
                    if residuals.isna().any().any():
                        continue

                    rpm_err = residuals["rpm"].max()
                    angle_err = residuals["pitchDegrees"].max()
                    flight_time_err = residuals["flightTimeSeconds"].max()

                    if (
                        rpm_err <= max_rpm_error
                        and angle_err <= max_angle_error
                        and flight_time_err <= max_tof_error
                        and rpm_err < best_max_rpm_err
                    ):
                        best_max_rpm_err = rpm_err
                        best_max_angle_err = angle_err
                        best_max_tof_err = flight_time_err
                        best_distance = distance

                if best_distance is None:
                    break

                print(
                    f"Dropped point at {best_distance} meters. (rpm_err={
                        best_max_rpm_err:.2f} angle_err={
                        best_max_angle_err:.2f} tof_err={best_max_tof_err:.2f})"
                )
                filtered_df = filtered_df[filtered_df.index != best_distance]

            filtered_df_to_write = filtered_df

        evaluate(df, filtered_df_to_write)

        done = inquirer.confirm(message="Done?").execute()
        if done:
            break

    with open("../src/main/deploy/constants/comp/ShotMapsCleaned.json", "w+") as f:
        f.truncate()
        json.dump(convert_frame_to_shotmaps(filtered_df_to_write), f, indent=2)

    print("Wrote file to ShotMapsCleaned.json.")


if __name__ == "__main__":
    main()
