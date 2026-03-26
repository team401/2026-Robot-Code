from util_data import AllianceStationID, Auto, AutoStatus, Pose, PublishingMode

unflipped_auto_suffix = " Blue"
flipped_auto_suffix = " Red"
unmirrored_auto_suffix = " Left"
mirrored_auto_suffix = " Right"

autos_file_path = "../src/main/deploy/constants/comp/Autos.json"

class DataManager:

    def __init__(self):
        self.python_executable = "python"
        self.build_autos_script_path = "../auto_generator/build_autos.py"

        self.current_publishing_mode: PublishingMode = PublishingMode.SIM
        self.custom_auto_environment: str = None
        self.custom_build_arguments: str = None


        self.last_set_robot_pose = Pose(0.0, 0.0, 0.0)
        self.starting_pose: Pose = Pose(4.0, 7.4, 0.0)
        self.selected_alliance_station: AllianceStationID = AllianceStationID.Red1

        self.run_simulation_gui: bool = False
        self.run_simulation_driver_station: bool = False

        self.auto_status: AutoStatus = AutoStatus.Idle

        self.autos: list = []
        from nt_manager import NTManager
        self.nt_manager = NTManager(self)
        self.team_number = 401

        from sim_manager import SimManager

        self.simulator_script = "bash run_sim.bash"
        # TODO: Look into if this is the correct arg and if not fix it
        self.simulator_script_args = []
        self.sim_manager = SimManager(self, self.get_sim_command)

    def get_sim_args(self) -> str:
        args_list = self.simulator_script_args.copy()
        # TODO: These should be fixed to be the proper arguments
        if self.run_simulation_gui:
            args_list.append("--gui")
        if self.run_simulation_driver_station:
            args_list.append("--driver-station")
        return " ".join(args_list)

    def get_sim_command(self) -> str:
        sim_args = self.get_sim_args()
        return f"{self.simulator_script} {sim_args}"

    def get_build_arguments(self) -> str:
        if self.custom_build_arguments is not None:
            return self.custom_build_arguments
        else:
            args_list = []
            if self.current_publishing_mode == PublishingMode.ROBOT:
                args_list.append("--robot")
            elif self.current_publishing_mode == PublishingMode.SIM:
                args_list.append("--sim")

            if self.custom_auto_environment is not None:
                args_list.append(f"--env={self.custom_auto_environment}")

            return " ".join(args_list)

    def get_build_command(self) -> str:
        build_arguments = self.get_build_arguments()
        return f"{self.python_executable} {self.build_autos_script_path} {build_arguments}"

    def rebuild_autos(self) -> None:
        command = self.get_build_command()
        import os
        os.system(command)

    def load_autos_from_file(self) -> None:
        import json
        with open(autos_file_path, "r", encoding="utf-8") as f:
            autos_data = json.load(f)
            autos = []
            autos_map = autos_data.get("autos", {})
            for auto_name, auto_info in autos_map.items():
                can_be_mirrored = auto_info.get("canBeMirrored", True)
                should_be_flipped = auto_info.get("shouldBeFlipped", True)
                auto = Auto(name=auto_name, can_be_mirrored=can_be_mirrored, should_be_flipped=should_be_flipped)
                autos.append(auto)
            self.autos = autos

    def _get_auto_variants(self, auto: Auto) -> list[str]:
        variants = []
        name = auto.name
        auto_unflipped_suffix = (unflipped_auto_suffix if auto.should_be_flipped else "")
        auto_unmirrored_suffix = (unmirrored_auto_suffix if auto.can_be_mirrored else "")
        variants.append(name + auto_unflipped_suffix + auto_unmirrored_suffix)
        if auto.can_be_mirrored:
            variants.append(name + auto_unflipped_suffix + mirrored_auto_suffix)
            if auto.should_be_flipped:
                variants.append(name + flipped_auto_suffix + mirrored_auto_suffix)
        if auto.should_be_flipped:
            variants.append(name + flipped_auto_suffix + auto_unmirrored_suffix)
        return variants

    def get_all_auto_names(self) -> list[str]:
        autos = []
        for auto in self.autos:
            for variant in self._get_auto_variants(auto):
                autos.append(variant)
        return autos

    def get_auto_status(self) -> AutoStatus:
        if self.nt_manager.is_auto_enabled():
            return AutoStatus.Running
        else:
            return AutoStatus.Idle

    def set_selected_auto_command_name(self, name: str) -> None:
        self.nt_manager.set_selected_auto_command_name(name)

    def get_selected_alliance_station(self) -> AllianceStationID:
        return self.nt_manager.get_selected_alliance_station()

    def set_selected_alliance_station(self, station: AllianceStationID) -> None:
        self.nt_manager.set_selected_alliance_station(station)

    def update(self):
        self.nt_manager.update()

    def run_auto(self) -> None:
        self.nt_manager.start_auto()

    def stop_auto(self) -> None:
        self.nt_manager.stop_auto()

    def is_simulator_running(self) -> bool:
        return self.sim_manager.is_simulator_running()


    def shutdown(self) -> None:
        if self.is_simulator_running():
            self.sim_manager.stop_simulator()

        self.nt_manager.nt_inst.stopClient()

    def get_robot_pose(self) -> Pose:
        return self.nt_manager.get_robot_pose()
