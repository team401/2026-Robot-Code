package frc.robot.auto;

import coppercore.parameter_tools.json.annotations.JSONExclude;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction.AutoActionData;
import frc.robot.auto.AutoParameters.AutoParameter;
import frc.robot.subsystems.drive.DriveCoordinator;

public class Auto {

  private AutoAction auto;

  @JSONExclude public AutoParameters parameters;

  @JSONExclude public AutoAction.AutoActionData data;

  @JSONExclude public boolean haveSetupAutoAction = false;

  public Auto() {
    parameters = new AutoParameters();
  }

  public void setData(DriveCoordinator driveCoordinator) {
    data = new AutoActionData(driveCoordinator, this);
  }

  public void registerParameter(AutoParameter<?> parameter) {
    parameters.registerParameter(parameter);
  }

  public void registerParameters(AutoParameter<?>... parameters) {
    for (var parameter : parameters) {
      registerParameter(parameter);
    }
  }

  public void setupAutoAction() {
    if (data == null) {
      throw new IllegalStateException("Auto data has not been set. Call setData() first.");
    }
    auto.setupAction(data);
    haveSetupAutoAction = true;
  }

  public void publishParameters() {
    if (!haveSetupAutoAction) {
      throw new IllegalStateException(
          "Auto parameters have not been set up. Call setupParameters() first.");
    }
    parameters.publishToDashboard();
  }

  public Command toCommand() {
    if (data == null) {
      throw new IllegalStateException("Auto data has not been set. Call setData() first.");
    }
    if (!haveSetupAutoAction) {
      throw new IllegalStateException(
          "Auto Action has not been set up yet. Call setupAutoAction() first.");
    }
    return auto.toCommand(data);
  }

  public AutoParameter<?> getParameter(String name) {
    return parameters.getParameter(name);
  }
}
