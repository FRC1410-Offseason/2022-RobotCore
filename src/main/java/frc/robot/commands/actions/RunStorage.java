package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;


public class RunStorage extends CommandBase {
    private final Storage storage;
    private final double speed;

    public RunStorage(Storage storage, double speed) {
        this.storage = storage;
        this.speed = speed;
        addRequirements(storage);
    }

    @Override
    public void initialize() {
        storage.runStorage(speed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        storage.runStorage(0);
    }
}
