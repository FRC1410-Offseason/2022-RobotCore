package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.controllers.Axis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;

import static frc.robotmap.Constants.INTAKE_FORWARD_SPEED;
import static frc.robotmap.Constants.STORAGE_INTAKE_SPEED;


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
		if (axis.getDeadzoned() != 0) {
			storage.setIntaking(true);
		} else {
			storage.setIntaking(false);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		intake.setSpeed(0);
		storage.runStorage(0);
	}
}
