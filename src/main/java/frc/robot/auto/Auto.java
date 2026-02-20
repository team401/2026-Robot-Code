package frc.robot.auto;

import coppercore.parameter_tools.json.annotations.JSONExclude;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction.AutoActionData;
import frc.robot.auto.AutoParameters.AutoParameter;
import frc.robot.subsystems.drive.DriveCoordinator;

public class Auto {

    public AutoAction auto;

    @JSONExclude
    public AutoParameters parameters;

    @JSONExclude
    public AutoAction.AutoActionData data;

    @JSONExclude
    public boolean haveSetupParameters = false;

    public Auto() {
        parameters = new AutoParameters();
    }

    public void setData(DriveCoordinator driveCoordinator) {
        data = new AutoActionData(driveCoordinator, this);
    }

    public void setupParameters() {
        if (data == null) {
            throw new IllegalStateException("Auto data has not been set. Call setData() first.");
        }
        auto.setupParameters(data);
        haveSetupParameters = true;
    }

    public void publishParameters() {
        if (!haveSetupParameters) {
            throw new IllegalStateException("Auto parameters have not been set up. Call setupParameters() first.");
        }
        parameters.publishToDashboard();
    }

    public Command toCommand() {
        if (data == null) {
            throw new IllegalStateException("Auto data has not been set. Call setData() first.");
        }
        return auto.toCommand(data);
    }

    public AutoParameter<?> getParameter(String name) {
        return parameters.getParameter(name);
    }
}