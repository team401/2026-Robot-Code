import math
import ntcore

from data_manager import DataManager
from time import sleep

from util_data import AllianceStationID, Pose


class NTManager:

    def __init__(self, data_manager: DataManager, address="127.0.0.1", port=5810):

        self.data_manager = data_manager
        self.nt_inst = ntcore.NetworkTableInstance.getDefault()
        self.nt_inst.stopClient()
        self.nt_inst.setServer(server_name=address, port=port)
        # self.nt_inst.setServerTeam(team)
        self.nt_inst.startClient4("python-auto-tester")


        prefix = "/SmartDashboard/AutoTesting/"


        self.robot_enabled_entry = self.nt_inst.getEntry(prefix + "RobotEnabled")
        self.auto_enabled_entry = self.nt_inst.getEntry(prefix + "AutoEnabled")

        self.auto_selector_entry = self.nt_inst.getEntry("/SmartDashboard/Auto Choices/selected")

        self.alliance_station_selected_entry = self.nt_inst.getEntry("/SmartDashboard/AutoTesting/AllianceStation/selected")
        self.alliance_station_active_entry = self.nt_inst.getEntry("/SmartDashboard/AutoTesting/AllianceStation/active")

        self.start_auto_entry = self.nt_inst.getEntry(prefix + "StartAuto")
        self.stop_auto_entry = self.nt_inst.getEntry(prefix + "StopAuto")
        self.enable_robot_entry = self.nt_inst.getEntry(prefix + "EnableRobot")
        self.disable_robot_entry = self.nt_inst.getEntry(prefix + "DisableRobot")

        self.set_robot_pos_entry = self.nt_inst.getEntry(prefix + "SetPos")

        self.starting_pos_x_entry = self.nt_inst.getEntry(prefix + "SetPosX")
        self.starting_pos_y_entry = self.nt_inst.getEntry(prefix + "SetPosY")
        self.starting_pos_theta_entry = self.nt_inst.getEntry(prefix + "SetPosTheta")

        self.robot_pos_x_entry = self.nt_inst.getEntry("/AdvantageKit/RealOutputs/Odometry/Robot/x")
        self.robot_pos_y_entry = self.nt_inst.getEntry("/AdvantageKit/RealOutputs/Odometry/Robot/y")
        self.robot_pos_theta_entry = self.nt_inst.getEntry("/AdvantageKit/RealOutputs/Odometry/Robot/theta")

    def get_robot_pose(self) -> Pose:
        x = self.robot_pos_x_entry.getDouble(0.0)
        y = self.robot_pos_y_entry.getDouble(0.0)
        theta = self.robot_pos_theta_entry.getDouble(0.0)
        return Pose(x, y, math.degrees(theta))

    def set_starting_pos(self, pos: Pose) -> None:
        self.starting_pos_x_entry.setDouble(pos.x)
        self.starting_pos_y_entry.setDouble(pos.y)
        self.starting_pos_theta_entry.setDouble(pos.rotation)

    def set_robot_pos(self) -> None:
        self.set_starting_pos(self.data_manager.starting_pose)
        self.data_manager.last_set_robot_pose = self.data_manager.starting_pose.copy()
        self.set_robot_pos_entry.setBoolean(True)

    def enable_robot(self) -> None:
        self.enable_robot_entry.setBoolean(True)

    def disable_robot(self) -> None:
        self.disable_robot_entry.setBoolean(True)

    def start_auto(self) -> None:
        self.set_robot_pos_entry.setBoolean(True)
        self.set_robot_pos_entry.setBoolean(False)
        self.start_auto_entry.setBoolean(True)
        # self.enable_robot()

    def stop_auto(self) -> None:
        # self.disable_robot()
        self.stop_auto_entry.setBoolean(True)

    def is_robot_enabled(self) -> bool:
        return self.robot_enabled_entry.getBoolean(False)

    def is_auto_enabled(self) -> bool:
        return self.auto_enabled_entry.getBoolean(False)

    def get_selected_auto(self) -> str:
        return self.auto_selector_entry.getString("")

    def select_auto(self, auto_name: str) -> None:
        self.auto_selector_entry.setString(auto_name)

    def get_selected_alliance_station(self) -> AllianceStationID:
        station_str = self.alliance_station_active_entry.getString("Data Not Found")
        match (station_str):
            case "Blue 1":
                return AllianceStationID.Blue1
            case "Blue 2":
                return AllianceStationID.Blue2
            case "Blue 3":
                return AllianceStationID.Blue3
            case "Red 1":
                return AllianceStationID.Red1
            case "Red 2":
                return AllianceStationID.Red2
            case "Red 3":
                return AllianceStationID.Red3
        return AllianceStationID.Unknown

    def set_selected_alliance_station(self, alliance_station: AllianceStationID):
        station_str = "Unknown"
        match (alliance_station):
            case AllianceStationID.Red1:
                station_str = "Red 1"
            case AllianceStationID.Red2:
                station_str = "Red 2"
            case AllianceStationID.Red3:
                station_str = "Red 3"
            case AllianceStationID.Blue1:
                station_str = "Blue 1"
            case AllianceStationID.Blue2:
                station_str = "Blue 2"
            case AllianceStationID.Blue3:
                station_str = "Blue 3"
        self.alliance_station_selected_entry.setString(station_str)


    def update(self) -> None:
        pass

