import argparse
from app import AutoTestingUtilApp
from data_manager import DataManager
from util_data import PublishingMode

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Utility for building and publishing autos.")
    parser.add_argument("--custom-build-args", type=str, help="Custom arguments to pass to the auto build process.")
    parser.add_argument("--publishing-mode", type=str, choices=[mode.name for mode in PublishingMode], help="Set the publishing mode for the auto build process.")
    parser.add_argument("--custom-auto-environment", type=str, help="Set a custom auto environment to be used during the auto build process.")
    parser.add_argument("--sim-gui", action="store_true", help="Whether to launch the auto testing simulation GUI when running simulation.")
    parser.add_argument("--sim-ds", action="store_true", help="Whether to launch the driver station when running simulation.")
    return parser.parse_args()

def apply_args_to_data_manager(args: argparse.Namespace, data_manager: DataManager) -> None:
    # BUILD ARGS
    if args.custom_build_args:
        data_manager.custom_build_arguments = args.custom_build_args
    if args.publishing_mode:
        data_manager.current_publishing_mode = PublishingMode[args.publishing_mode]
    if args.custom_auto_environment:
        data_manager.custom_auto_environment = args.custom_auto_environment

    # SIM ARGS
    data_manager.run_simulation_gui = args.sim_gui
    data_manager.run_simulation_driver_station = args.sim_ds

def main() -> None:
    data_manager = DataManager()

    args = parse_args()
    apply_args_to_data_manager(args, data_manager)

    data_manager.load_autos_from_file()

    app = AutoTestingUtilApp(data_manager)

    app.run()

if __name__ == "__main__":
    main()
