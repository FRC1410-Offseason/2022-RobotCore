package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;

import static frc.robotmap.Constants.SHOOTER_ARM_DOWN_SPEED;


public class LowerShooterArmConstant extends CommandBase {

	private final ShooterArm shooterArm;

	public LowerShooterArmConstant(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		addRequirements(shooterArm);
	}

	@Override
	public void execute() {
		shooterArm.manualSet(SHOOTER_ARM_DOWN_SPEED);
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
