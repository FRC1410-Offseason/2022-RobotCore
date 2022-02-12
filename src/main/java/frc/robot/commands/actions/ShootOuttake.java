package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterArm;
import frc.robot.subsystems.Storage;

import static frc.robotmap.Constants.*;

public class ShootOuttake extends CommandBase {

	private final Shooter shooter;
	private final ShooterArm shooterArm;
	private final Storage storage;

	public ShootOuttake(Shooter shooter, ShooterArm shooterArm, Storage storage) {
		this.shooter = shooter;
		this.shooterArm = shooterArm;
		this.storage = storage;
		addRequirements(shooter, shooterArm, storage);
	}

	@Override
	public void initialize() {
		shooterArm.setGoalPos(SHOOTER_ARM_OUTTAKE_ANGLE);
		shooter.setSpeeds(SHOOTER_OUTTAKE_SPEED);
	}

	@Override
	public void execute() {
		if (shooterArm.isAtTarget() && shooter.isAtTarget()) {
			storage.runStorage(STORAGE_RUN_SPEED);
		}
	}

	@Override
	public boolean isFinished() {
		return shooter.getShotCount() != 0;
	}

	@Override
	public void end(boolean interrupted) {
		shooterArm.setGoalPos(SHOOTER_ARM_RESTING_ANGLE);
		shooter.setSpeeds(0);
		storage.runStorage(0);
	}
}
