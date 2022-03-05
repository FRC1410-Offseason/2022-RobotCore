package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;

import static frc.robotmap.Constants.SHOOTER_ARM_DOWN_SPEED;


public class LowerShooterArm extends CommandBase {

	private final ShooterArm shooterArm;

	public LowerShooterArm(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		addRequirements(shooterArm);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		shooterArm.set(SHOOTER_ARM_DOWN_SPEED);
	}

	@Override
	public boolean isFinished() {
		return shooterArm.getLowerLimit();
	}

	@Override
	public void end(boolean interrupted) {
		shooterArm.set(0);
	}
}
