package frc.robot.auto;

import coppercore.parameter_tools.json.annotations.JsonSubtype;
import coppercore.parameter_tools.json.annotations.JsonType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoParameters.AutoParameter;
import frc.robot.auto.drive.AutoPilotAction;
import frc.robot.auto.general.Deadline;
import frc.robot.auto.general.Parallel;
import frc.robot.auto.general.Race;
import frc.robot.auto.general.RoutineCall;
import frc.robot.auto.general.Sequence;
import frc.robot.subsystems.drive.DriveCoordinator;

@JsonType(
    property = "type",
    subtypes = {
      // General actions
      @JsonSubtype(clazz = RoutineCall.class, name = "RoutineCall"),
      @JsonSubtype(clazz = Deadline.class, name = "Deadline"),
      @JsonSubtype(clazz = Sequence.class, name = "Sequence"),
      @JsonSubtype(clazz = Parallel.class, name = "Parallel"),
      @JsonSubtype(clazz = Race.class, name = "Race"),
      // Drive actions
      @JsonSubtype(clazz = AutoPilotAction.class, name = "AutoPilotAction"),
    })
public abstract class AutoAction {

  public String type;

  public record AutoActionData(DriveCoordinator driveCoordinator, Auto auto) {}

  public static AutoParameter getParameter(Auto auto, String name) {
    return auto.parameters.parameters.get(name);
  }

  public void setupParameters(AutoActionData data) {}

  public abstract Command toCommand(AutoActionData data);
}
