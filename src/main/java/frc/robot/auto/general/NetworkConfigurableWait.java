package frc.robot.auto.general;

import static edu.wpi.first.units.Units.Seconds;

import coppercore.parameter_tools.json.annotations.AfterJsonLoad;
import coppercore.parameter_tools.json.annotations.JSONExclude;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.AutoAction;
import java.util.HashMap;
import java.util.Map;
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

  @JSONExclude
  private static final Map<String, LoggedNetworkNumber> nameToLoggedNetworkNumber = new HashMap<>();

  @AfterJsonLoad
  public void initializeNetworkNumber() {
    this.waitTimeSeconds =
        nameToLoggedNetworkNumber.computeIfAbsent(
            name,
            name ->
                new LoggedNetworkNumber(
                    "NetworkConfigurableWait/" + name,
                    defaultDelay != null ? defaultDelay.in(Seconds) : 0.0));
  }

  @Override
  public Command toCommand(AutoActionContext data) {
    return new DeferredCommand(() -> new WaitCommand(Seconds.of(waitTimeSeconds.get())), Set.of());
  }
}
