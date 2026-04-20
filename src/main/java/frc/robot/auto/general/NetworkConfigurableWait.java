package frc.robot.auto.general;

import static edu.wpi.first.units.Units.Seconds;

import coppercore.parameter_tools.json.annotations.AfterJsonLoad;
import coppercore.parameter_tools.json.annotations.JSONExclude;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.AutoAction;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// Class written with the help of Github Copilot auto-completions

/**
 * A NetworkConfigurableWait is a Wait action that can be configured in real-time through the
 * network.
 */
public class NetworkConfigurableWait extends AutoAction {
  private String name;
  private Time defaultDelay;
  @JSONExclude private LoggedNetworkNumber waitTimeSeconds;
  @JSONExclude private static final Set<String> usedNames = new java.util.HashSet<>();

  @AfterJsonLoad
  public void initializeNetworkNumber() {
    if (usedNames.contains(name)) {
      throw new IllegalArgumentException(
          "NetworkConfigurableWait name must be unique. Duplicate name: " + name);
    }
    usedNames.add(name);

    this.waitTimeSeconds =
        new LoggedNetworkNumber(
            "NetworkConfigurableWait/" + name,
            defaultDelay != null ? defaultDelay.in(Seconds) : 0.0);
  }

  @Override
  public Command toCommand(AutoActionContext data) {
    return new DeferredCommand(() -> new WaitCommand(Seconds.of(waitTimeSeconds.get())), Set.of());
  }
}
