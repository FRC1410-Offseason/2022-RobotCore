package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeFlipper;

import static frc.robotmap.Tuning.*;

public class RunIntakeFlipper extends CommandBase {

	private final IntakeFlipper intakeFlipper;

	public RunIntakeFlipper(IntakeFlipper intakeFlipper) {
		this.intakeFlipper = intakeFlipper;
		addRequirements(intakeFlipper);
	}

	@Override
	public void execute() {
		if (intakeFlipper.getDesiredPosition()) {

			// If we want to be extended
			if (Math.abs(intakeFlipper.getEncoderPosition() - INTAKE_DOWN_POSITION) > INTAKE_IS_FINISHED) {
				// If we are not within our tolerance for being extended
				// We have to run the PID
				intakeFlipper.setPIDSpeed(INTAKE_DOWN_POSITION);
			} else {
				// If we are within our tolerance for being extended, we can stop moving
				intakeFlipper.setSpeed(0);
			}
		} else {
			// If we want to be retracted
			if (Math.abs(intakeFlipper.getEncoderPosition() - INTAKE_UP_POSITION) > INTAKE_IS_FINISHED) {
				// If we are not within our tolerance for being retracted
				// We have to run the PID
				intakeFlipper.setPIDSpeed(INTAKE_UP_POSITION);
			} else {
				// If we are within our tolerance for being retracted, we can stop moving
				intakeFlipper.setSpeed(0);
			}
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		// If the command ends we don't want to keep moving the flipper
		intakeFlipper.setSpeed(0);
	}
}
