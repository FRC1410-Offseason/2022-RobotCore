package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.controllers.Axis;
import frc.robot.subsystems.Intake;

import static frc.robotmap.Constants.INTAKE_FORWARD_SPEED;


public class RunIntake extends CommandBase {

	private final Intake intake;
	private final Axis axis;


	public RunIntake(Intake intake, Axis axis) {
		this.intake = intake;
		this.axis = axis;
		addRequirements(intake);
	}

	@Override
	public void execute() {
		intake.setSpeed(axis.getDeadzoned());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
