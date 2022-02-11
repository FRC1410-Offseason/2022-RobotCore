package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;


public class SetShooterArmAngle extends CommandBase {

	private final ShooterArm shooterArm;
	private final double angle;

	public SetShooterArmAngle(ShooterArm shooterArm, double angle) {
		this.shooterArm = shooterArm;
		this.angle = angle;
		addRequirements(this.shooterArm);
	}

	@Override
	public void initialize() {
		shooterArm.setGoalPos(angle);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
