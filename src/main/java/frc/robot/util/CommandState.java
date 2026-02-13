package frc.robot.util;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;

// Copilot used to help write the docs for this class

/**
 * A State implementation that wraps and drives a {@code Command} for the duration of the state.
 *
 * <p>This state forwards lifecycle events from the state machine to an optional {@code Command}
 * instance:
 *
 * <ul>
 *   <li>onEntry(...) &nbsp;&nbsp;→ calls {@code command.initialize()} if a command is set
 *   <li>periodic(...) &nbsp;→ calls {@code command.execute()} each periodic update while active
 *   <li>onExit(...) &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;→ calls {@code command.end(false)} if a command
 *       is set
 * </ul>
 *
 * <p>The presence of a {@code Command} is optional; if no command is supplied (i.e. {@code null}),
 * the state performs no command-related actions. The {@code Command} is not scheduled with any
 * external scheduler by this class — only its lifecycle methods are invoked directly by the state
 * machine callbacks.
 *
 * @param <World> the type of the world/context object passed to state lifecycle methods
 * @see #getCommand()
 * @see #setCommand(Command)
 */
public class CommandState<World> extends State<World> {

  private Command command;

  /**
   * Constructs a new CommandState with the specified name and associated Command.
   *
   * <p>The provided name is forwarded to the superclass constructor and is used to identify this
   * state. The command parameter is stored and represents the Command associated with this state.
   *
   * @param name the human-readable name for this state; forwarded to the superclass
   * @param command the Command to associate with this state
   */
  public CommandState(String name, Command command) {
    super(name);
    this.command = command;
  }

  /**
   * Called when this state is entered by the state machine.
   *
   * <p>Invoked during a transition into this state. If a non-null {@code command} is associated
   * with the state, its {@code initialize()} method is invoked to perform any setup required before
   * the command runs. If {@code command} is null, no action is taken.
   *
   * @param stateMachine the state machine that triggered the transition
   * @param world the current world/context provided to the state
   */
  @Override
  protected void onEntry(StateMachine<World> stateMachine, World world) {
    if (command != null) {
      command.initialize();
    }
  }

  /**
   * Periodic update hook invoked by the owning state machine for this state. If this state has an
   * associated command (i.e. the internal {@code command} reference is non-null), that command will
   * be executed once per invocation of this method.
   *
   * <p>Implementations should keep work performed here short and non-blocking to avoid delaying the
   * state machine's periodic cycle.
   *
   * @param stateMachine the state machine that owns and invoked this state; may be used to request
   *     transitions or query global state
   * @param world the shared world/context object provided to states for reading or modifying
   */
  @Override
  protected void periodic(StateMachine<World> stateMachine, World world) {
    if (command != null) {
      command.execute();
    }
  }

  /**
   * Called when this state is exited by the state machine.
   *
   * <p>If there is an associated command, this will terminate that command by calling its
   * end(false) method (indicating a normal completion rather than an interruption). A null-check is
   * performed so there is no action when no command is present.
   *
   * @param stateMachine the StateMachine that is performing the transition away from this state
   * @param world the shared World/context instance associated with the state machine
   */
  @Override
  protected void onExit(StateMachine<World> stateMachine, World world) {
    if (command != null) {
      command.end(false);
    }
  }
}
