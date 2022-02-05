package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.input.Axis;
import frc.robot.subsystems.Intake;

import static frc.robotmap.Constants.INTAKE_FORWARD_SPEED;


public class RunIntake extends CommandBase {

	private final Intake intake;
	private final Axis axis;

	public RunIntake(Intake intake) {
		this.intake = intake;
		this.axis = null;
		addRequirements(intake);
	}

	public RunIntake(Intake intake, Axis axis) {
		this.intake = intake;
		this.axis = axis;
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		if (axis != null) {
			intake.setSpeed(axis.getDeadzoned());
		} else {
			intake.setSpeed(INTAKE_FORWARD_SPEED);
		}
	}

	@Override
	public void execute() {
		if (axis != null) {
			intake.setSpeed(axis.getDeadzoned());
		}
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		intake.setSpeed(0);
	}
}
