package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;

import static frc.robotmap.Constants.SHOOTER_ARM_UP_SPEED;


public class RaiseShooterArm extends CommandBase {

	private final ShooterArm shooterArm;

	public RaiseShooterArm(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		addRequirements(shooterArm);
	}

	@Override
	public void execute() {
		shooterArm.set(SHOOTER_ARM_UP_SPEED);
	}

	@Override
	public boolean isFinished() {
		return shooterArm.getUpperLimit();
	}

	@Override
	public void end(boolean interrupted) {
		shooterArm.set(0);
	}
}
