package frc.robot.commands.looped;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterArm;
import frc.robot.subsystems.Storage;

import static frc.robotmap.Constants.*;

public class OuttakeHandler extends CommandBase {

	private final Shooter shooter;
	private final ShooterArm shooterArm;
	private final Storage storage;

	private final Timer outtakeTimer = new Timer();

	private boolean internalOuttakeStarted = false;

	public OuttakeHandler(Shooter shooter, ShooterArm shooterArm, Storage storage) {
		this.shooter = shooter;
		this.shooterArm = shooterArm;
		this.storage = storage;

		addRequirements(this.shooter, this.shooterArm, this.storage);
	}

	@Override
	public void initialize() {
		outtakeTimer.reset();
	}

	@Override
	public void execute() {
		//If we need to outtake
		if (storage.getOuttakeFlag()) {
			//If there is a ball in the second storage slot
			if (storage.getCurrentState().getSlot2().getBallPresent()) {
				//TODO: Set flag to tell shooter what to do during the shooting sequence
			} else {
				//Else, we can outtake immediately
				if (!internalOuttakeStarted) {
					//Set the shooter arm to the outtake position
					shooterArm.setGoal(SHOOTER_ARM_OUTTAKE_ANGLE);

					//Set the speed of the shooter flywheels
					shooter.setSpeeds(SHOOTER_OUTTAKE_SPEED);

					//Start the timer
					outtakeTimer.start();
				}

				if (outtakeTimer.get() < STORAGE_OUTTAKE_TIME) {
					//Run the storage and outtake the ball
					storage.runStorage(STORAGE_OUTTAKE_SPEED);

				} else {
					//If the outtake sequence has been running for long enough, we are good to stop it and reset everything for next time
					//Stop the timer and reset it for next time
					outtakeTimer.stop();
					outtakeTimer.reset();

					//Have the shooter arm go back down to the resting position
					shooterArm.setGoal(0);

					//Spin down the shooter motors
					shooter.setSpeeds(0);

					//Stop running the storage
					storage.runStorage(0);
				}
			}
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
