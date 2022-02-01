package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;


public class RunStorage extends CommandBase {
    private final Storage storage;
    private final double speed;

    public RunStorage(Storage storage, double speed) {
        this.storage = storage;
        this.speed = speed;
        addRequirements(this.storage);
    }

    @Override
    public void execute() {
        storage.runStorage(speed);
    }
}
