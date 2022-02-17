package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;


public class SetShooterArmBrake extends CommandBase {

	private final ShooterArm shooterArm;

	public SetShooterArmBrake(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		addRequirements(shooterArm);
	}

	@Override
	public void initialize() {
		shooterArm.setBrake();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
