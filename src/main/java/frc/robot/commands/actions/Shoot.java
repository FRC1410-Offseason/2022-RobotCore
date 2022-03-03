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

	private final int rpm;
	private final int num;

	/**
	 * Shoot the cargo!
	 * @param shooter the shooter subsystem
	 * @param shooterArm the shooterArm subsystem
	 * @param storage the storage subsystem
	 * @param rpm the rpm that we want to shoot at
	 * @param num the number of cargo that we want to shoot
	 */
	public Shoot(Shooter shooter, ShooterArm shooterArm, Storage storage, int rpm, int num) {
		this.shooter = shooter;
		this.shooterArm = shooterArm;
		this.storage = storage;

		this.rpm = rpm;
		this.num = num;

		addRequirements(shooter, shooterArm, storage);
	}

	@Override
	public void initialize() {
		// Reset the shot count of the shooter
		shooter.resetShotCount();

		//Set the position of the shooter arm to the shooting angle
		shooterArm.setGoal(SHOOTER_ARM_MAX_ANGLE);

		//Get the NEOs on the shooter spinning up to our desired RPM
		shooter.setSpeeds(rpm);
	}

	@Override
	public void execute() {
		// If the shooter is up to speed and the arm is at the correct position, we are good to shoot
		if (shooterArm.isAtTarget() && shooter.isAtTarget()) {
			storage.runStorage(STORAGE_SHOOT_SPEED);
		}
	}

	@Override
	public boolean isFinished() {
		// The shooter automatically keeps track of when a ball passes through the flywheels,
		// so we just have to compare that to the number of cargo that we want to shoot
		return shooter.getShotCount() < num;
	}

	@Override
	public void end(boolean interrupted) {
		//Reset everything and put the mechanisms back into their resting states
		shooter.setSpeeds(0);
		shooterArm.setGoal(SHOOTER_ARM_RESTING_ANGLE);
		shooter.resetShotCount();

		//Reset the state of the storage
		storage.getCurrentState().resetSlot1();
		storage.getCurrentState().resetSlot2();
	}
}
