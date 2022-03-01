package frc.robot.framework.scheduler.task;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.framework.subsystem.SubsystemRegistry;

public class CommandTask extends Task {

	private final Command command;
	private CommandState state = CommandState.PENDING;

	public CommandTask(Command command) {
		this.command = command;
	}

    public Command getCommand() {
        return command;
    }

    @Override
    public void initialize() {
        SubsystemRegistry.interruptAllNecessaryLocks(this);
        SubsystemRegistry.applyAllNecessaryLocks(this);
    }

	@Override
	public void execute() {
		switch (state) {
			case PENDING: {
				command.initialize();
				state = CommandState.RUNNING;
				break;
			}

			case RUNNING: {
				command.execute();

				if (command.isFinished()) {
					state = CommandState.FINISHED;
				}

				break;
			}

            case INTERRUPTION_PENDING: {
				state = CommandState.INTERRUPTION_PENDING;
				command.end(true);
				break;
			}
		}
	}

	@Override
	public boolean isFinished() {
		return state == CommandState.FINISHED;
	}

    @Override
	public void end() {
        command.end(false);
        state = CommandState.PENDING;

        SubsystemRegistry.releaseAllNecessaryLocks(this);
	}

	public void interrupt() {
        command.end(true);
		state = CommandState.PENDING;

        SubsystemRegistry.releaseAllNecessaryLocks(this);
	}

    @Override
    public boolean isValidToExecute() {
        System.out.println("Check for if " + this.getCommand() + " is the highest priority: " + SubsystemRegistry.isHighestPriority(this));
        return SubsystemRegistry.isHighestPriority(this);
    }

	private enum CommandState {
		PENDING,
		RUNNING,
		INTERRUPTION_PENDING,
		FINISHED
	}
}