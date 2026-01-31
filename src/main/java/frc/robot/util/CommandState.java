package frc.robot.util;

import coppercore.controls.state_machine.State;
import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;

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
