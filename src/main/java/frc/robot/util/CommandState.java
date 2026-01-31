package frc.robot.util;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A State implementation that wraps and drives a {@code Command} for the duration of the state.
 *
 * <p>This state forwards lifecycle events from the state machine to an optional {@code Command}
 * instance:
 * <ul>
 *   <li>onEntry(...) &nbsp;&nbsp;→ calls {@code command.initialize()} if a command is set</li>
 *   <li>periodic(...) &nbsp;→ calls {@code command.execute()} each periodic update while active</li>
 *   <li>onExit(...) &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;→ calls {@code command.end(false)} if a command is set</li>
 * </ul>
 *
 * <p>The presence of a {@code Command} is optional; if no command is supplied (i.e. {@code null}),
 * the state performs no command-related actions. The {@code Command} is not scheduled with any
 * external scheduler by this class — only its lifecycle methods are invoked directly by the
 * state machine callbacks.
 *
 * @param <World> the type of the world/context object passed to state lifecycle methods
 *
 * @see #getCommand()
 * @see #setCommand(Command)
 */
public class CommandState<World> extends State<World> {
    
    private Command command;

    public CommandState(String name) {
        this(name, null);
    }

    public CommandState(String name, Command command) {
        super(name);
        this.command = command;
    }

    public Command getCommand() {
        return command;
    }

    public void setCommand(Command command) {
        this.command = command;
    }

    @Override
    protected void onEntry(StateMachine<World> stateMachine, World world) {
        if (command != null) {
            command.initialize();
        }
    }

    @Override
    protected void periodic(StateMachine<World> stateMachine, World world) {
        if (command != null) {
            command.execute();
        }
    }

    @Override
    protected void onExit(StateMachine<World> stateMachine, World world) {
        if (command != null) {
            command.end(false);
        }
    }

}
