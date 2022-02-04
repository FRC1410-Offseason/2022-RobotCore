package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.Axis;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final Axis leftAxis;
    private final Axis rightAxis;

    public TankDrive(Drivetrain drivetrain, Axis leftAxis, Axis rightAxis) {
        this.drivetrain = drivetrain;
        this.leftAxis = leftAxis;
        this.rightAxis = rightAxis;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.tankDrive(leftAxis.getDeadzoned(), rightAxis.getDeadzoned());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}