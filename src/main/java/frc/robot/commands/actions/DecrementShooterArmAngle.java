package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;

import static frc.robotmap.Constants.*;


public class DecrementShooterArmAngle extends CommandBase {

	private final ShooterArm shooterArm;

	public DecrementShooterArmAngle(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		addRequirements(shooterArm);
	}

	@Override
	public void execute() {
		if (shooterArm.getGoalPos() - SHOOTER_ARM_ANGLE_OFFSET > SHOOTER_ARM_RESTING_ANGLE) {
			shooterArm.setGoalPos(shooterArm.getGoalPos() - SHOOTER_ARM_ANGLE_OFFSET);
		}
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
