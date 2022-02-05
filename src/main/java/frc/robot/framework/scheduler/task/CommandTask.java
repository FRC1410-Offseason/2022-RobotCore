package frc.robot.framework.scheduler.task;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandTask implements Task {

	private final Command command;
	private CommandState state = CommandState.PENDING;

	public CommandTask(Command command) {
		this.command = command;
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
					command.end(false);
				}

				break;
			}

			case INTERRUPTION_PENDING: {
				state = CommandState.FINISHED;
				command.end(false);
				break;
			}
		}
	}

	@Override
	public boolean isFinished() {
		return state == CommandState.FINISHED;
	}

	public void interrupt() {
		state = CommandState.INTERRUPTION_PENDING;
	}

	private enum CommandState {
		PENDING,
		RUNNING,
		INTERRUPTION_PENDING,
		FINISHED
	}
}
