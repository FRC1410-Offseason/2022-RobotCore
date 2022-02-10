package frc.robot.framework.scheduler.task;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.framework.scheduler.SubsystemRegistry;

public class CommandTask implements Task {

	private final Command command;
	private CommandState state = CommandState.PENDING;

	public CommandTask(Command command) {
		this.command = command;
	}

	@Override
	public void execute() {
		switch (state) {
			// The phase responsible for initializing the command and starting its run cycle.
			case PENDING: {
				if (!ownsAllLocks()) break;
				for (Subsystem requirement : command.getRequirements()) {
					SubsystemRegistry.applyLock(requirement, command);
				}

				command.initialize();

				state = CommandState.RUNNING;
				break;
			}

			// Runs the command, checking subsystem requirements and whether the command is finished.
			case RUNNING: {
				if (!ownsAllLocks()) {
					state = CommandState.SUSPENDED;
					break;
				}

				command.execute();

				if (command.isFinished()) {
					state = CommandState.FINISHED;
					command.end(false);
				}

				break;
			}

			// Checks if the command should still be suspended, resuming if necessary.
			case SUSPENDED: {
				if (ownsAllLocks()) {
					state = CommandState.PENDING;
				}

				break;
			}

			// Forces the command to finish and ends the Task.
			case INTERRUPTION_PENDING: {
				state = CommandState.FINISHED;
				command.end(false);
				break;
			}

			case FINISHED: {
				for (Subsystem requirement : command.getRequirements()) {
					SubsystemRegistry.releaseLock(requirement, command);
				}
			}
		}
	}

	private boolean ownsAllLocks() {
		for (Subsystem requirement : command.getRequirements()) {
			if (!SubsystemRegistry.ownsLock(requirement, command)) return false;
		}

		return true;
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
		SUSPENDED,
		INTERRUPTION_PENDING,
		FINISHED
	}
}
