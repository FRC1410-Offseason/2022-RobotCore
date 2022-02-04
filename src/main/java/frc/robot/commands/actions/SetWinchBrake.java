package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Winch;


public class SetWinchBrake extends CommandBase {
    private final Winch winch;

    public SetWinchBrake(Winch winch) {
        this.winch = winch;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(winch);
    }

    @Override
    public void initialize() {
        this.winch.lock();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }
}