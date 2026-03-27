from util_data import AllianceStationID, AutoStatus, Pose
from data_manager import DataManager
from textual.app import App, ComposeResult
from textual.containers import HorizontalGroup, VerticalGroup, VerticalScroll, Grid, Vertical
from textual.widgets import Button, Footer, Header, OptionList, Label, RichLog, Switch, Input
from textual.reactive import reactive
from textual.widgets.option_list import Option

class FloatingInput(Vertical):
    """A reusable floating label input widget with floating label."""

    CSS = """
        .floating-label {
            dock: top;
            color: grey;
            offset-y: 1;
            transition: offset 0.2s, color 0.2s;
        }

        .floating-label.floated {
            offset-y: -1;
            color: blue;
            bold: true;
        }
    """

    value = reactive("")  # track input value

    def __init__(self, label_text: str, **kwargs):
        super().__init__()
        self.label_text = label_text
        self.kwargs = kwargs

    def compose(self) -> ComposeResult:
        # Label that will float
        self.label = Label(self.label_text, classes="floating-label")
        yield self.label
        # Input field
        self.input = Input(**self.kwargs)
        yield self.input

    def on_mount(self) -> None:
        self.input.focus()
        self._update_label_position()

    # Listen for InputChanged events
    def on_input_changed(self, event: Input.Changed) -> None:
        self.value = event.value
        self._update_label_position()

    def _update_label_position(self):
        if self.input.value:
            self.label.add_class("floated")
        else:
            self.label.remove_class("floated")


class AllianceStationSelector(VerticalGroup):
    """UI component to select the alliance station."""

    CSS = """

    """

    def __init__(self, data_manager: DataManager):
        super().__init__()
        self.data_manager = data_manager
        self.selected_station = None

    def compose(self) -> ComposeResult:
        # two rows of station buttons (Red row, Blue row) so we can control vertical spacing precisely
        with HorizontalGroup(id="station_row_red"):
            yield Button("Red 1", id="red1", variant="default")
            yield Button("Red 2", id="red2", variant="default")
            yield Button("Red 3", id="red3", variant="default")
        with HorizontalGroup(id="station_row_blue"):
            yield Button("Blue 1", id="blue1", variant="default")
            yield Button("Blue 2", id="blue2", variant="default")
            yield Button("Blue 3", id="blue3", variant="default")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        station_map = {
            "red1": AllianceStationID.Red1,
            "red2": AllianceStationID.Red2,
            "red3": AllianceStationID.Red3,
            "blue1": AllianceStationID.Blue1,
            "blue2": AllianceStationID.Blue2,
            "blue3": AllianceStationID.Blue3,
            "unknown": AllianceStationID.Unknown,
        }
        station_id = station_map.get(event.button.id)
        if station_id:
            self.data_manager.nt_manager.set_selected_alliance_station(station_id)

    def on_mount(self):
        self.set_interval(0.1, self.update_selected_station)

    def update_selected_station(self):
        selected = self.data_manager.nt_manager.get_selected_alliance_station()
        station_id = selected.name.lower() if selected else None

        if station_id != self.selected_station:
            self.selected_station = station_id
            for button in self.query(Button):
                button.variant = "primary" if button.id == station_id else "default"


class AutoSelector(VerticalScroll):
    """UI component to select an autonomous routine."""
    def __init__(self, data_manager: DataManager):
        super().__init__()
        self.data_manager = data_manager

    def compose(self) -> ComposeResult:
        yield Label("Autos", id="autos_label")
        self.auto_list = OptionList(
            *[Option(name) for name in self.data_manager.get_all_auto_names()],
            id="auto_selector"
        )
        yield self.auto_list

    def update_auto_list(self):
        # self.auto_list.clear_options()
        # for name in self.data_manager.get_all_auto_names():
        #     self.auto_list.add_option(Option(name))
        # self.auto_list.recompose()
        self.recompose()

    def on_option_list_option_selected(self, event: OptionList.OptionSelected) -> None:
        self.data_manager.nt_manager.select_auto(event.option.prompt)



class AutoControls(VerticalGroup):
    """UI controls for running and stopping autonomous routines."""
    def __init__(self, data_manager: DataManager):
        super().__init__()
        self.data_manager = data_manager

    def compose(self) -> ComposeResult:
        with HorizontalGroup():
            yield Button("Run Auto", id="run_auto")
            yield Button("Stop Auto", id="stop_auto")
        self.auto_status = Label("Auto Status: Idle", id="auto_status")
        yield self.auto_status

    def on_mount(self):
        self.set_interval(0.1, self.update_auto_status)

    def update_auto_status(self):
        status = self.data_manager.get_auto_status()
        status_str = "Idle" if status == AutoStatus.Idle else "Running"
        self.auto_status.update(f"Auto Status: {status_str}")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "run_auto":
            self.data_manager.run_auto()
        elif event.button.id == "stop_auto":
            self.data_manager.stop_auto()

class RobotPosControls(VerticalGroup):
    """UI controls for robot position adjustments."""
    CSS = """
        Grid {
            grid-size: 2;
            padding: 1;
        }
    Button {
        width: 10;
        height: 3;
        content-align: center middle;
    }
    Input {
        width: 12;
    }
    #start_pos_label {
        padding: 1 0 0 1;
    }
    """

    def __init__(self, data_manager: DataManager):
        super().__init__()
        self.data_manager = data_manager
        self.x_step = 0.1
        self.y_step = 0.1
        self.rotation_step = 5.0

    def compose(self) -> ComposeResult:

        self.pose_label = Label("Random starting position", id="robot_pos")
        yield self.pose_label

        # 2 columns x 3 rows: (X+/X-), (Y+/Y-), (Rot+/Rot-)
        with HorizontalGroup(id="robot_pos_set_inc"):
            yield Button("X +", id="robot_set_x_up")
            yield Button("Y +", id="robot_set_y_up")
            yield Button("Rotation +", id="robot_set_rotation_up")
        with HorizontalGroup(id="robot_pos_set_dec"):
            yield Button("X -", id="robot_set_x_down")
            yield Button("Y -", id="robot_set_y_down")
            yield Button("Rotation -", id="robot_set_rotation_down")

         # show current starting position
        self.start_pos_label = Label(self._format_starting_pose(), id="start_pos_label")
        yield self.start_pos_label

        yield Switch("Auto Set Starting Position", id="auto_set_starting_pos")
        yield Button("Set Robot Position", id="set_robot_pos")

        # Starting position inputs
        with HorizontalGroup():
            x_input = FloatingInput("X:", placeholder=str(self.data_manager.starting_pose.x), id="start_x_input")
            x_input.value = str(self.data_manager.starting_pose.x)
            yield x_input
            y_input = FloatingInput("Y:", placeholder=str(self.data_manager.starting_pose.y), id="start_y_input")
            y_input.value = str(self.data_manager.starting_pose.y)
            yield y_input
            r_input = FloatingInput("Rot:", placeholder=str(self.data_manager.starting_pose.rotation), id="start_rot_input")
            r_input.value = str(self.data_manager.starting_pose.rotation)
            yield r_input

            yield Button("Apply Start Pos", id="apply_start_pos")

    def on_mount(self):
        self.set_interval(0.01, self.update_auto_set_starting_pos)
        self.set_interval(0.01, self.update_start_pos_label)
        self.set_interval(0.01, self.update_robot_pose)

    def update_robot_pose(self):
        self.pose_label.update(self._format_robot_pose())

    def _format_robot_pose(self) -> str:
        pose = self.data_manager.get_robot_pose()
        return f"Robot Pose: x={pose.x:.2f}, y={pose.y:.2f}, rot={pose.rotation:.1f}°"

    def update_auto_set_starting_pos(self):
        auto_set = self.query_one("#auto_set_starting_pos", Switch).value
        if auto_set and self.data_manager.last_set_robot_pose != self.data_manager.starting_pose:
            self.data_manager.nt_manager.set_robot_pos()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        delta_map = {
            "robot_set_x_up": Pose(self.x_step, 0, 0),
            "robot_set_x_down": Pose(-self.x_step, 0, 0),
            "robot_set_y_up": Pose(0, self.y_step, 0),
            "robot_set_y_down": Pose(0, -self.y_step, 0),
            "robot_set_rotation_up": Pose(0, 0, self.rotation_step),
            "robot_set_rotation_down": Pose(0, 0, -self.rotation_step),
        }
        if event.button.id in delta_map:
            self.data_manager.starting_pose = self.data_manager.starting_pose.add(delta_map[event.button.id])
            x_input = self.query_one("#start_x_input", Input)
            x_input.value = str(self.data_manager.starting_pose.x)
            y_input = self.query_one("#start_y_input", Input)
            y_input.value = str(self.data_manager.starting_pose.y)
            r_input = self.query_one("#start_rot_input", Input)
            r_input.value = str(self.data_manager.starting_pose.rotation)
            x_input.refresh()
            y_input.refresh()
            r_input.refresh()

        if event.button.id == "apply_start_pos":
            # read inputs and set starting_pose (do not force network update)
            try:
                x_str = self.query_one("#start_x_input", Input).value or self.query_one("#start_x_input", Input).placeholder
                y_str = self.query_one("#start_y_input", Input).value or self.query_one("#start_y_input", Input).placeholder
                r_str = self.query_one("#start_rot_input", Input).value or self.query_one("#start_rot_input", Input).placeholder
                x = float(x_str)
                y = float(y_str)
                r = float(r_str)
                self.data_manager.starting_pose = Pose(x, y, r)
            except Exception:
                # ignore conversion errors
                pass

        if self.query_one("#auto_set_starting_pos", Switch).value or event.button.id == "set_robot_pos":
            self.data_manager.nt_manager.set_robot_pos()

    def update_start_pos_label(self):
        # refresh label to show current starting pose
        self.start_pos_label.update(self._format_starting_pose())

    def _format_starting_pose(self) -> str:
        sp = self.data_manager.starting_pose
        return f"Starting Pos: x={sp.x:.2f}, y={sp.y:.2f}, rot={sp.rotation:.1f}°"


class SimulatorControls(VerticalGroup):
    """UI controls for the simulator with logs."""
    def __init__(self, data_manager: DataManager):
        super().__init__()
        self.data_manager = data_manager

    def compose(self) -> ComposeResult:
        with HorizontalGroup():
            yield Button("Start Simulator", id="start_simulator")
            yield Button("Stop Simulator", id="stop_simulator")
            self.sim_status_label = Label("Simulator Status: Stopped", id="simulator_status")
            yield self.sim_status_label
        # smaller simulator-specific log on the right column
        self.sim_logs = RichLog(id="sim_logs_right")
        yield self.sim_logs

    def on_mount(self):
        self.set_interval(0.1, self.update_simulator_status)
        self.set_interval(0.1, self.update_simulator_output)

    def update_simulator_status(self):
        status = "Running" if self.data_manager.is_simulator_running() else "Stopped"
        self.sim_status_label.update(f"Simulator Status: {status}")

    def update_simulator_output(self):
        out = self.data_manager.sim_manager.get_simulator_output()
        if not out:
            return
        # get_simulator_output returns a str; split into lines preserving endings
        for line in out:
            self.sim_logs.write(line)

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "start_simulator":
            self.data_manager.sim_manager.start_simulator()
        elif event.button.id == "stop_simulator":
            self.data_manager.sim_manager.stop_simulator()


class AutoTestingUtilApp(App):
    """Main dashboard app with clean panel layout."""
    BINDINGS = [
        ("l", "load_autos", "Load Autos"),
        ("r", "rebuild_autos", "Rebuild Autos")
    ]

    def __init__(self, data_manager: DataManager):
        super().__init__()
        self.data_manager = data_manager
    CSS = """
    #main_grid {
        grid-size: 3;
        grid-columns: 40% 35% 25%;
        padding: 1;
    }
    #auto_selector {
        height: 18;
    }
    #sim_logs_right {
        height: 20;
    }
    /* station grid replaced by two HorizontalGroup rows inside AllianceStationSelector */
    #left_col AllianceStationSelector {
        height: 6;
        padding: 0;
    }
    #left_col AllianceStationSelector HorizontalGroup {
        margin: 0;
        padding: 0;
        height: 3;
    }
    #left_col {
        grid-size: 2;
        grid-rows: auto auto;
        grid-columns: 1fr;
        padding: 0;
    }
    #left_col > * {
        margin: 0;
        padding: 0;
    }
    /* align rules removed because Textual CSS does not support 'align-content' or 'align-items' */
    #autos_label {
        padding: 1 0 0 1;
    }
    .floating-label {
        dock: top;
        color: grey;
        offset-y: 1;
        transition: offset 0.2s, color 0.2s;
    }
    .floating-label.floated {
        offset-y: 1;
        color: blue;
    }
    """

    def compose(self) -> ComposeResult:
        yield Header()

        # Main 3-column grid: left controls, center autos, right simulator
        with Grid(id="main_grid"):
            with VerticalGroup(id="left_col"):
                yield AllianceStationSelector(self.data_manager)
                yield RobotPosControls(self.data_manager)

            with VerticalGroup(id="center_col"):
                yield AutoSelector(self.data_manager)
                yield AutoControls(self.data_manager)

            with VerticalGroup(id="right_col"):
                yield SimulatorControls(self.data_manager)

        yield Footer()

    def on_mount(self):
        self.set_interval(0.1, self.data_manager.update)

    def action_load_autos(self):
        self.data_manager.load_autos_from_file()
        self.query_one(AutoSelector).update_auto_list()

    def action_rebuild_autos(self):
        self.data_manager.rebuild_autos()
        self.action_load_autos()

    def on_shutdown(self):
        self.data_manager.shutdown()
