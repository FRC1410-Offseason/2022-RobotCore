package frc.robot.commands.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NetworkTables;
import frc.robot.subsystems.Storage;

import static frc.robotmap.Constants.STORAGE_RUN_SPEED;


public class SetStorageSpeed extends CommandBase {

	private final Storage storage;
	private final Timer timer = new Timer();
	private double time = -1;

	public SetStorageSpeed(Storage storage) {
		this.storage = storage;
		// addRequirements(storage);
	}

	public SetStorageSpeed(Storage storage, double time) {
		this.storage = storage;
		this. time = time;
	}

	@Override
	public void initialize() {
		storage.runStorage(STORAGE_RUN_SPEED);
		if (time != -1) {
			timer.start();
		}
		NetworkTables.setStorageSpeed(STORAGE_RUN_SPEED);
	}

	@Override
	public boolean isFinished() {
		if (time != -1) {
			return timer.get() > time;
		} else {
			return true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		storage.runStorage(0);
		NetworkTables.setStorageSpeed(0);
	}
}
