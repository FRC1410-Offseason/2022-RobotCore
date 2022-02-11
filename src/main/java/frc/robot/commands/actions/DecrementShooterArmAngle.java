package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;

import static frc.robotmap.Constants.SHOOTER_ARM_ANGLE_OFFSET;


public class DecrementShooterArmAngle extends CommandBase {

	private final ShooterArm shooterArm;

	public DecrementShooterArmAngle(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		addRequirements(this.shooterArm);
	}

	@Override
	public void initialize() {
		shooterArm.setGoal(shooterArm.getGoal().position - SHOOTER_ARM_ANGLE_OFFSET);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
