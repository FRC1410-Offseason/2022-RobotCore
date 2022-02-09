package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;

import static frc.robotmap.Constants.STORAGE_RUN_SPEED;


public class RunStorage extends CommandBase {

	private final Storage storage;

	public RunStorage(Storage storage) {
		this.storage = storage;
		addRequirements(storage);
	}

	@Override
	public void initialize() {
		storage.runStorage(STORAGE_RUN_SPEED);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
