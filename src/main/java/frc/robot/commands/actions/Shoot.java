package frc.robot.commands.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

import static frc.robotmap.Constants.*;
import static frc.robotmap.Tuning.*;

public class Shoot extends CommandBase {

	private final Shooter shooter;
	private final Storage storage;
	private final Timer timer = new Timer();
	private boolean storageIsRunning = false;

	private final int rpm;

	/**
	 * Shoot the cargo!
	 * @param shooter the shooter subsystem
	 * @param shooterArm the shooterArm subsystem
	 * @param storage the storage subsystem
	 * @param rpm the rpm that we want to shoot at
	 * @param num the number of cargo that we want to shoot
	 */
	public Shoot(Shooter shooter, Storage storage, int rpm) {
		this.shooter = shooter;
		this.storage = storage;

		this.rpm = rpm;

		addRequirements(shooter, storage);
	}

	@Override
	public void initialize() {
		//Get the NEOs on the shooter spinning up to our desired RPM
		shooter.setSpeeds(rpm);
	}

	@Override
	public void execute() {
		// If the shooter is up to speed and the arm is at the correct position, we are good to shoot
		if (shooter.isAtTarget()) {
			storage.runStorage(STORAGE_SHOOT_SPEED);
			if (!storageIsRunning) {
				timer.start();
				timer.reset();
				storageIsRunning = true;
			}
		}
	}

	@Override
	public boolean isFinished() {
		// The shooter automatically keeps track of when a ball passes through the flywheels,
		// so we just have to compare that to the number of cargo that we want to shoot
		return timer.get() > SHOOT_STORAGE_DURATION;
	}

	@Override
	public void end(boolean interrupted) {
		//Reset everything and put the mechanisms back into their resting states
		shooter.setSpeeds(0);

		//Reset the state of the storage
		storage.getCurrentState().resetSlot1();
		storage.getCurrentState().resetSlot2();
	}
}
