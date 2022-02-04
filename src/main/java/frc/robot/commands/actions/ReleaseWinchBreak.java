package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Winch;


public class ReleaseWinchBreak extends CommandBase {
    private final Winch winch;

    public ReleaseWinchBreak(Winch winch) {
        this.winch = winch;
        addRequirements(winch);
    }

    @Override
    public void initialize() {
        this.winch.unlock();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }
}
