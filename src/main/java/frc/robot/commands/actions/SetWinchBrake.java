package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Winch;


public class SetWinchBrake extends CommandBase {
    private final Winch winch;

    public SetWinchBrake(Winch winch) {
        this.winch = winch;
        addRequirements(winch);
    }

    @Override
    public void initialize() {
        winch.lock();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}