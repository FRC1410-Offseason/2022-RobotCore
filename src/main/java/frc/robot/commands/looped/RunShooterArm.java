package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;

import static frc.robotmap.Tuning.*;

public class RunShooterArm extends CommandBase {

	private final ShooterArm shooterArm;

	public RunShooterArm(ShooterArm shooterArm) {
		this.shooterArm = shooterArm;
		addRequirements(this.shooterArm);
	}

	@Override
	public void execute() {
		if (Math.abs(shooterArm.getEncoderPosition() - shooterArm.getGoal()) > SHOOTER_ARM_IS_FINISHED) {
			// If we are not within our is finished tolerance
			shooterArm.runPIDExecute();
		} else {
			// If we are within our tolerance
			shooterArm.setVoltage(0);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		shooterArm.setVoltage(0);
	}
}
