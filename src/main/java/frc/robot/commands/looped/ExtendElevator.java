package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.Axis;
import frc.robot.subsystems.Elevator;

public class ExtendElevator extends CommandBase {
    private final Elevator elevator;
    private final Axis axis;

    public ExtendElevator(Elevator elevator, Axis axis) {
        this.elevator = elevator;
        this.axis = axis;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.runLeftElevator(axis.getDeadzoned());
        elevator.runRightElevator(axis.getDeadzoned());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
