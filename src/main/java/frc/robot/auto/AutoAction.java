package frc.robot.auto;

import coppercore.parameter_tools.json.annotations.JsonSubtype;
import coppercore.parameter_tools.json.annotations.JsonType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CoordinationLayer;
import frc.robot.auto.coordinationLayer.ClimbHangAction;
import frc.robot.auto.coordinationLayer.ClimbSearchAction;
import frc.robot.auto.coordinationLayer.DeployIntakeAction;
import frc.robot.auto.coordinationLayer.StartShooting;
import frc.robot.auto.coordinationLayer.StopShooting;
import frc.robot.auto.coordinationLayer.StowIntakeAction;
import frc.robot.auto.drive.AutoPilotAction;
import frc.robot.auto.drive.FollowPathPlannerPath;
import frc.robot.auto.drive.StopDriveAction;
import frc.robot.auto.drive.XBasedAutoPilotAction;
import frc.robot.auto.general.AutoReference;
import frc.robot.auto.general.Deadline;
import frc.robot.auto.general.NetworkConfigurableWait;
import frc.robot.auto.general.Parallel;
import frc.robot.auto.general.Print;
import frc.robot.auto.general.Race;
import frc.robot.auto.general.Sequence;
import frc.robot.auto.general.Wait;
import frc.robot.subsystems.drive.DriveCoordinator;

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
      @JsonSubtype(clazz = NetworkConfigurableWait.class, name = "NetworkConfigurableWait"),
      // Drive actions
      @JsonSubtype(clazz = AutoPilotAction.class, name = "AutoPilotAction"),
      @JsonSubtype(clazz = XBasedAutoPilotAction.class, name = "XBasedAutoPilotAction"),
      @JsonSubtype(clazz = StopDriveAction.class, name = "StopDriveAction"),
      @JsonSubtype(clazz = FollowPathPlannerPath.class, name = "FollowPathPlannerPath"),
      // Coordination layer actions
      @JsonSubtype(clazz = DeployIntakeAction.class, name = "DeployIntakeAction"),
      @JsonSubtype(clazz = StowIntakeAction.class, name = "StowIntakeAction"),
      @JsonSubtype(clazz = ClimbSearchAction.class, name = "ClimbSearchAction"),
      @JsonSubtype(clazz = ClimbHangAction.class, name = "ClimbHangAction"),
      @JsonSubtype(clazz = StartShooting.class, name = "StartShooting"),
      @JsonSubtype(clazz = StopShooting.class, name = "StopShooting"),
    })
public abstract class AutoAction {

  public String type;

  public record AutoActionContext(
      DriveCoordinator driveCoordinator,
      CoordinationLayer coordinationLayer,
      Autos autos,
      boolean flipped,
      boolean mirrored) {

    public AutoActionContext(
        DriveCoordinator driveCoordinator, CoordinationLayer coordinationLayer, Autos autos) {
      this(driveCoordinator, coordinationLayer, autos, false, false);
    }

    public AutoActionContext flip() {
      return new AutoActionContext(driveCoordinator, coordinationLayer, autos, !flipped, mirrored);
    }

    public AutoActionContext mirror() {
      return new AutoActionContext(driveCoordinator, coordinationLayer, autos, flipped, !mirrored);
    }
  }

  public abstract Command toCommand(AutoActionContext data);
}
