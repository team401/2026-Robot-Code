package frc.robot.auto.general;

import java.util.stream.Stream;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoAction;

public class Deadline extends AutoAction {
    
    public AutoAction deadline;

    public AutoAction[] others;

    @Override
    public Command toCommand(AutoActionData data) {
        return deadline.toCommand(data).deadlineFor(
            Stream.of(others).map(
                action -> action.toCommand(data)
            ).toArray(Command[]::new)
        );
    }

}
