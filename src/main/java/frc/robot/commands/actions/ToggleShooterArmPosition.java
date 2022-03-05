package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;

import static frc.robotmap.Constants.*;


public class ToggleShooterArmPosition extends CommandBase {

	private final ShooterArm shooterArm;

	public ToggleShooterArmPosition(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		addRequirements(shooterArm);
	}

	@Override
	public void initialize() {
		if (shooterArm.getLowerLimit()) {
			shooterArm.set(SHOOTER_ARM_UP_SPEED);
		} else {
			shooterArm.set(SHOOTER_ARM_DOWN_SPEED);
		}
	}

	@Override
	public void execute() {

	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
