package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;


public class RunStorageConstant extends CommandBase {

	private final Storage storage;
	private final double speed;

	public RunStorageConstant(Storage storage, double speed) {
		this.storage = storage;
		this.speed = speed;
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
