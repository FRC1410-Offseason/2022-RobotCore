package frc.robot.framework.scheduler.task;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandTask implements Task {

	private final Command command;
	private CommandState state = CommandState.PENDING;

	public CommandTask(Command command) {
		this.command = command;
	}

    public Command getCommand() {
        return command;
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
		}
	}

	@Override
	public boolean isFinished() {
		return state == CommandState.FINISHED;
	}

    @Override
	public void end() {
        state = CommandState.PENDING;
	}

	public void interrupt() {
        command.end(true);
		state = CommandState.FINISHED;
	}

	private enum CommandState {
		PENDING,
		RUNNING,
		INTERRUPTION_PENDING,
		FINISHED
	}
}