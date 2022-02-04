package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Winch;


public class SetWinchSpeed extends CommandBase {
    private final Winch winch;
    private final double speed;

    public SetWinchSpeed(Winch winch, double speed) {
        this.winch = winch;
        this.speed = speed;
        addRequirements(winch);
    }

    @Override
    public void initialize() {
        this.winch.runWinch(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        this.winch.runWinch(0);
    }
}
