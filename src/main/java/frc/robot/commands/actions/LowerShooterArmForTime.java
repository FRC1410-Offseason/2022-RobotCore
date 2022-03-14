package frc.robot.commands.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;

import static frc.robotmap.Constants.SHOOTER_ARM_DOWN_SPEED;


public class LowerShooterArmForTime extends CommandBase {

	private final ShooterArm shooterArm;
	private final Timer timer = new Timer();
	private final double time;

	public LowerShooterArmForTime(ShooterArm shooterArm, double time) {
		this.shooterArm = shooterArm;
		this.time = time;
	}

	@Override
	public void initialize() {
		timer.start();
		shooterArm.manualSet(SHOOTER_ARM_DOWN_SPEED);
	}

	@Override
	public boolean isFinished() {
		return timer.get() >= time;
	}

	@Override
	public void end(boolean interrupted) {
		shooterArm.manualSet(0);
	}
}
