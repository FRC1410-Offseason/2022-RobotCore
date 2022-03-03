package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;


public class RunStorage extends CommandBase {

	private final Storage storage;
	private final double speed;

	public RunStorage(Storage storage, double speed) {
		this.storage = storage;
		this.speed = speed;
		// each subsystem used by the command must be passed into the
		// addRequirements() method (which takes a vararg of Subsystem)
		addRequirements(storage);
	}

	@Override
	public void initialize() {
		storage.runStorage(speed);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		storage.runStorage(0);
	}
}
