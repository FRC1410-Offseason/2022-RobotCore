package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;


public class ReleaseShooterArmBrake extends CommandBase {

	private final ShooterArm shooterArm;

	public ReleaseShooterArmBrake(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		addRequirements(shooterArm);
	}

	@Override
	public void initialize() {
		shooterArm.releaseBrake();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
