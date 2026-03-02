package frc.robot.auto;

import coppercore.parameter_tools.json.annotations.JsonSubtype;
import coppercore.parameter_tools.json.annotations.JsonType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.drive.AutoPilotAction;
import frc.robot.auto.general.Deadline;
import frc.robot.auto.general.Parallel;
import frc.robot.auto.general.Print;
import frc.robot.auto.general.Race;
import frc.robot.auto.general.Sequence;
import frc.robot.auto.general.Wait;
import frc.robot.subsystems.drive.DriveCoordinator;
import frc.robot.util.ts.TypeScriptAppend;
import frc.robot.util.ts.TypeScriptMethod;

@TypeScriptAppend(
    value =
        "let _addCommandHook: ((command: NonNullable<AutoAction>) => void) | null = null;\n"
            + "export function setAddCommandHook(hook: (command: NonNullable<AutoAction>) => void)"
            + " { _addCommandHook = hook; }")
@TypeScriptMethod(
    name = "add",
    returnType = "this",
    body = {"_addCommandHook?.(this);", "return this;"},
    comment = "Adds this command to the current auto and returns itself for chaining.")
@JsonType(
    property = "type",
    subtypes = {
      // General actions
      @JsonSubtype(clazz = Deadline.class, name = "Deadline"),
      @JsonSubtype(clazz = Sequence.class, name = "Sequence"),
      @JsonSubtype(clazz = Parallel.class, name = "Parallel"),
      @JsonSubtype(clazz = Race.class, name = "Race"),
      @JsonSubtype(clazz = Wait.class, name = "Wait"),
      @JsonSubtype(clazz = Print.class, name = "Print"),
      // Drive actions
      @JsonSubtype(clazz = AutoPilotAction.class, name = "AutoPilotAction"),
    })
public abstract class AutoAction {

  public String type;

  public record AutoActionContext(DriveCoordinator driveCoordinator, Autos auto) {}

  public abstract Command toCommand(AutoActionContext data);
}
