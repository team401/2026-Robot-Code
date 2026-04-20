package frc.robot.auto.general;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import frc.robot.auto.AutoAction;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// Class written with the help of Github Copilot auto-completions

/**
 * A NetworkConfigurableWait is a Wait action that can be configured in real-time through the
 * network.
 */
public class NetworkConfigurableWait extends AutoAction {
  private final LoggedNetworkNumber waitTimeSeconds;
  private static final Set<String> usedNames = new java.util.HashSet<>();

  public NetworkConfigurableWait(String name, Time defaultDelay) {
    if (usedNames.contains(name)) {
      throw new IllegalArgumentException(
          "NetworkConfigurableWait name must be unique. Duplicate name: " + name);
    }
    usedNames.add(name);

    this.waitTimeSeconds =
        new LoggedNetworkNumber("NetworkConfigurableWait/" + name, defaultDelay.in(Seconds));
  }

  @Override
  public edu.wpi.first.wpilibj2.command.Command toCommand(AutoActionContext data) {
    return new edu.wpi.first.wpilibj2.command.WaitCommand(Seconds.of(waitTimeSeconds.get()));
  }
}
