package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;


public class SetShooterArmBrake extends CommandBase {

	private final ShooterArm shooterArm;

	public SetShooterArmBrake(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		// each subsystem used by the command must be passed into the
		// addRequirements() method (which takes a vararg of Subsystem)
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
