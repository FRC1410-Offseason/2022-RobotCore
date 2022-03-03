package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.controllers.Axis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;

public class RunIntake extends CommandBase {

	private final Intake intake;
	private final Storage storage;
	private final Axis axis;


	public RunIntake(Intake intake, Storage storage, Axis axis) {
		this.intake = intake;
		this.storage = storage;
		this.axis = axis;
		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.setSpeed(axis.getDeadzoned());
		storage.setIntaking(axis.getDeadzoned() != 0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		intake.setSpeed(0);
	}
}
