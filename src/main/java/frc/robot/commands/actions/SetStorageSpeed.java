package frc.robot.commands.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NetworkTables;
import frc.robot.subsystems.Storage;

import static frc.robotmap.Constants.STORAGE_RUN_SPEED;


public class SetStorageSpeed extends CommandBase {

	private final Storage storage;

	public SetStorageSpeed(Storage storage) {
		this.storage = storage;
		// addRequirements(storage);
	}

	@Override
	public void initialize() {
		storage.runStorage(STORAGE_RUN_SPEED);
		NetworkTables.setStorageSpeed(STORAGE_RUN_SPEED);
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		storage.runStorage(0);
		NetworkTables.setStorageSpeed(0);
	}
}
