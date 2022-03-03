package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;

import static frc.robotmap.Constants.SHOOTER_ARM_INTAKE_ANGLE;
import static frc.robotmap.Constants.SHOOTER_ARM_MAX_ANGLE;


public class ToggleShooterArmPosition extends CommandBase {

	private final ShooterArm shooterArm;

	public ToggleShooterArmPosition(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		addRequirements(this.shooterArm);
	}

	@Override
	public void initialize() {
		if (shooterArm.isAtTarget()) {
			if (shooterArm.getGoal() == SHOOTER_ARM_INTAKE_ANGLE) {
				shooterArm.setGoal(SHOOTER_ARM_MAX_ANGLE);
			} else {
				shooterArm.setGoal(SHOOTER_ARM_INTAKE_ANGLE);
			}
		}
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
