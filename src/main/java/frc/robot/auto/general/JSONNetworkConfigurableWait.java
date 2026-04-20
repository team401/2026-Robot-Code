package frc.robot.auto.general;

import static edu.wpi.first.units.Units.Seconds;

import coppercore.parameter_tools.json.helpers.JSONObject;
import edu.wpi.first.units.measure.Time;
import java.lang.reflect.Constructor;

public class JSONNetworkConfigurableWait extends JSONObject<NetworkConfigurableWait> {
  String name;
  Time defaultDelay;

  public JSONNetworkConfigurableWait(NetworkConfigurableWait networkConfigurableWait) {
    super(networkConfigurableWait);
  }

  @Override
  public NetworkConfigurableWait toJava() {
    return new NetworkConfigurableWait(name, defaultDelay != null ? defaultDelay : Seconds.zero());
  }

  /**
   * Gets the constructor of the json wrapper
   *
   * @return the json wrapper constructor
   */
  public static Constructor<JSONNetworkConfigurableWait> getConstructor()
      throws NoSuchMethodException {
    return JSONNetworkConfigurableWait.class.getConstructor(NetworkConfigurableWait.class);
  }
}
