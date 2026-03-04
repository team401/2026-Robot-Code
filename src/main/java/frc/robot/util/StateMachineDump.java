package frc.robot.util;

import coppercore.controls.state_machine.StateMachine;
import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

public class StateMachineDump {
  public static final String outputDir = "state_machine_graphs";

  /**
   * Write a graphviz file of this state machine when in simulation.
   *
   * @param basename - the name of the file to write to (without the .dot suffix)
   * @param stateMachine - the state machine to dump
   */
  public static void write(String basename, StateMachine<?> stateMachine) {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    try {
      PrintWriter pw = new PrintWriter(new File(outputDir + File.separator + basename + ".dot"));
      stateMachine.writeGraphvizFile(pw);
      pw.close();
    } catch (IOException ioe) {
      System.err.println(ioe);
    }
  }
}
