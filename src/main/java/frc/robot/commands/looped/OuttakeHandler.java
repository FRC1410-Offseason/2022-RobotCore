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
		// TODO: This timer shouldn't be needed anymore, probably have to refactor
		outtakeTimer.reset();
	}

	@Override
	public void execute() {
		// If we need to outtake
		if (storage.getOuttakeFlag()) {
			// If there is a ball in the second storage slot
			if (storage.getCurrentState().getSlot2().getBallPresent()) {
				// If there is a cargo in the way of us automatically outtaking, we have to tell the shooter to outtake the next time that we shoot
				shooter.queueOuttake();
			} else {
				// Else, we can outtake immediately
				if (!internalOuttakeStarted) {
					shooter.resetShotCount();
					// Update the flag to say that we have started outtaking
					internalOuttakeStarted = true;

					// Set the speed of the shooter flywheels
					shooter.setSpeeds(SHOOTER_OUTTAKE_SPEED);
				}

				if (shooter.getShotCount() < 1) {
					// Run the storage and outtake the ball
					storage.runStorage(STORAGE_OUTTAKE_SPEED);

				} else {
					// If the outtake sequence has been running for long enough, we are good to stop it and reset everything for next time
					// Reset the storage state because now the cargo is no longer in the robot
					storage.getCurrentState().resetSlot1();

					// Reset the started flag for next time
					internalOuttakeStarted = false;

					// Spin down the shooter motors
					shooter.setSpeeds(0);

					// Stop running the storage
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
