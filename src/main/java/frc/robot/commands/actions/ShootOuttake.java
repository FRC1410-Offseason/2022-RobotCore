package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterArm;
import frc.robot.subsystems.Storage;

import static frc.robotmap.Constants.*;

public class ShootOuttake extends CommandBase {

	private final Shooter shooter;
	private final Storage storage;

	public ShootOuttake(Shooter shooter, Storage storage) {
		this.shooter = shooter;
		this.storage = storage;
		addRequirements(shooter, storage);
	}

	@Override
	public void initialize() {
		// Spin up the shooter motors
		shooter.setSpeeds(SHOOTER_OUTTAKE_SPEED);

		// Reset the shooter's internal count
		shooter.resetShotCount();
	}

	@Override
	public void execute() {
		// If everything is ready then we can start outtaking
		if (shooter.isAtTarget()) {
			storage.runStorage(STORAGE_RUN_SPEED);
		}
	}

	@Override
	public boolean isFinished() {
		// The only situation in which this command would be used is when there is one ball that we have to shoot
		// Therefore we just have to check that we've shot the ball
		return shooter.getShotCount() != 0;
	}

	@Override
	public void end(boolean interrupted) {
		// Reset everything
		shooter.setSpeeds(0);
		storage.runStorage(0);
		shooter.resetShotCount();
	}
}
