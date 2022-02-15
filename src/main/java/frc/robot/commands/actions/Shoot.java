package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterArm;
import frc.robot.subsystems.Storage;

import static frc.robotmap.Constants.*;

public class Shoot extends CommandBase {

	private final Shooter shooter;
	private final ShooterArm shooterArm;
	private final Storage storage;

	private final int numCargo;

	private final int rpm;
	private final int num;

	public Shoot(Shooter shooter, ShooterArm shooterArm, Storage storage, int rpm, int num) {
		this.shooter = shooter;
		this.shooterArm = shooterArm;
		this.storage = storage;

		numCargo = storage.getCurrentState().getNumCargo();

		this.rpm = rpm;
		this.num = num;

		addRequirements(shooter, shooterArm, storage);
	}

	@Override
	public void initialize() {
		shooter.resetShotCount();
		shooterArm.setGoalPos(SHOOTER_ARM_MAX_ANGLE);
		shooter.setSpeeds(rpm);
	}

	@Override
	public void execute() {
		if (shooterArm.isAtTarget() && shooter.isAtTarget()) {
			storage.runStorage(STORAGE_SHOOT_SPEED);
		}
	}

	@Override
	public boolean isFinished() {
		return shooter.getShotCount() < num;
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setSpeeds(0);
		shooterArm.setGoalPos(SHOOTER_ARM_RESTING_ANGLE);
		shooter.resetShotCount();
		storage.getCurrentState().resetSlot1();
		storage.getCurrentState().resetSlot2();
	}
}
