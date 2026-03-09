package frc.robot.auto;

import coppercore.parameter_tools.json.annotations.JsonSubtype;
import coppercore.parameter_tools.json.annotations.JsonType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CoordinationLayer;
import frc.robot.auto.coordinationLayer.ClimbHangAction;
import frc.robot.auto.coordinationLayer.ClimbSearchAction;
import frc.robot.auto.coordinationLayer.DeployIntakeAction;
import frc.robot.auto.coordinationLayer.StowIntakeAction;
import frc.robot.auto.drive.AutoPilotAction;
import frc.robot.auto.drive.StopDriveAction;
import frc.robot.auto.drive.XBasedAutoPilotAction;
import frc.robot.auto.general.AutoReference;
import frc.robot.auto.general.Deadline;
import frc.robot.auto.general.Parallel;
import frc.robot.auto.general.Print;
import frc.robot.auto.general.Race;
import frc.robot.auto.general.Sequence;
import frc.robot.auto.general.Wait;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.util.ts.PythonAppend;
import frc.robot.util.ts.PythonMethod;

@PythonAppend(
    value =
        "\n_add_command_hook: Callable[[Any], None] | None = None\n\n\n"
            + "def set_add_command_hook(hook: Callable[[Any], None]) -> None:\n"
            + "    global _add_command_hook\n"
            + "    _add_command_hook = hook\n\n\n"
            + "def _call_hook(obj: Any) -> None:\n"
            + "    if _add_command_hook is not None:\n"
            + "        _add_command_hook(obj)\n\n\n"
            + "def _to_dict(obj: Any) -> Any:\n"
            + "    \"\"\"Recursively convert an object to a JSON-serializable dict.\"\"\"\n"
            + "    if obj is None:\n"
            + "        return None\n"
            + "    if hasattr(obj, 'to_dict'):\n"
            + "        return obj.to_dict()\n"
            + "    return obj\n")
@PythonMethod(
    name = "add",
    returnType = "self",
    body = {"_call_hook(self)", "return self"},
    comment = "Adds this command to the current auto and returns itself for chaining.")
@JsonType(
    property = "type",
    subtypes = {
      // General actions
      @JsonSubtype(clazz = AutoReference.class, name = "AutoReference"),
      @JsonSubtype(clazz = Deadline.class, name = "Deadline"),
      @JsonSubtype(clazz = Sequence.class, name = "Sequence"),
      @JsonSubtype(clazz = Parallel.class, name = "Parallel"),
      @JsonSubtype(clazz = Race.class, name = "Race"),
      @JsonSubtype(clazz = Wait.class, name = "Wait"),
      @JsonSubtype(clazz = Print.class, name = "Print"),
      // Drive actions
      @JsonSubtype(clazz = AutoPilotAction.class, name = "AutoPilotAction"),
      @JsonSubtype(clazz = XBasedAutoPilotAction.class, name = "XBasedAutoPilotAction"),
      @JsonSubtype(clazz = StopDriveAction.class, name = "StopDriveAction"),
      // Coordination layer actions
      @JsonSubtype(clazz = DeployIntakeAction.class, name = "DeployIntakeAction"),
      @JsonSubtype(clazz = StowIntakeAction.class, name = "StowIntakeAction"),
      @JsonSubtype(clazz = ClimbSearchAction.class, name = "ClimbSearchAction"),
      @JsonSubtype(clazz = ClimbHangAction.class, name = "ClimbHangAction")
    })
public abstract class AutoAction {

  public String type;

  public record AutoActionContext(
      DriveCoordinator driveCoordinator, CoordinationLayer coordinationLayer, Autos autos) {}

  public abstract Command toCommand(AutoActionContext data);
}
