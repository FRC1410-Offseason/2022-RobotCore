package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;

import static frc.robotmap.Constants.STORAGE_REVERSE_SPEED;


public class ReverseStorage extends CommandBase {

	private final Storage storage;

	public ReverseStorage(Storage storage) {
		this.storage = storage;
		addRequirements(storage);
	}

	@Override
	public void initialize() {
		storage.runStorage(STORAGE_REVERSE_SPEED);
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		storage.runStorage(0);
	}
}
