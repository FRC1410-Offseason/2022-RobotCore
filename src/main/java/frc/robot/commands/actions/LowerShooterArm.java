package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;


public class LowerShooterArm extends CommandBase {

	private final ShooterArm shooterArm;

	public LowerShooterArm(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		addRequirements(shooterArm);
	}

	@Override
	public void initialize() {
		shooterArm.retract();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
