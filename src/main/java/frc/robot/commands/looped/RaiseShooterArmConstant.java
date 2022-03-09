package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;

import static frc.robotmap.Constants.SHOOTER_ARM_UP_SPEED;


public class RaiseShooterArmConstant extends CommandBase {

	private final ShooterArm shooterArm;

	public RaiseShooterArmConstant(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		addRequirements(shooterArm);
	}

	@Override
	public void execute() {
		shooterArm.set(SHOOTER_ARM_UP_SPEED);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		shooterArm.set(0);
	}
}
