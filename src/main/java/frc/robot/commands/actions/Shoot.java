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

	/**
	 * Shoot the cargo!
	 * @param shooter the shooter subsystem
	 * @param storage the storage subsystem
	 */
	public Shoot(Shooter shooter, Storage storage) {
		this.shooter = shooter;
		this.storage = storage;

//		 addRequirements(shooter, storage);
		addRequirements(storage);
	}

	@Override
	public void initialize() {
		timer.reset();
		storage.runStorage(STORAGE_SHOOT_SPEED);
		timer.start();
	}

	@Override
	public boolean isFinished() {
		// Runs for 2 seconds once storage starts running
		return timer.get() > SHOOT_STORAGE_DURATION;
	}

	@Override
	public void end(boolean interrupted) {
		//Reset everything and put the mechanisms back into their resting states
		shooter.setSpeeds(0);
		storage.runStorage(0);
		System.out.println("shoot (storage) ended");
		//Reset the state of the storage
		storage.getCurrentState().resetSlot1();
		storage.getCurrentState().resetSlot2();
	}
}
