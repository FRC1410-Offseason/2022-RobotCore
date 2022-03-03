package frc.robot.commands.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;

public class RunStorageForTime extends CommandBase {

    private final Storage storage;
    private final Timer timer = new Timer();
    private final double seconds;
	private final double speed;

    public RunStorageForTime(Storage storage, double seconds, double speed) {
        this.storage = storage;
        this.seconds = seconds;
		this.speed = speed;
    }

    @Override
    public void initialize() {
        timer.start();
        storage.runStorage(speed);
    }

    @Override
    public boolean isFinished() {
		return timer.get() >= seconds;
	}

    @Override
    public void end(boolean interrupted) {
        storage.runStorage(0);
    }
}
