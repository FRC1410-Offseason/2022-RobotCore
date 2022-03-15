package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;


public class RaiseShooterArm extends CommandBase {

	private final ShooterArm shooterArm;

	public RaiseShooterArm(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		addRequirements(shooterArm);
	}

	@Override
	public void initialize() {
		shooterArm.extend();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
