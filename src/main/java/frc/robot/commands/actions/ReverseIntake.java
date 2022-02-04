package frc.robot.commands.actions;

import static frc.robotmap.Constants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.input.Axis;
import frc.robot.subsystems.Intake;


public class ReverseIntake extends CommandBase {
    private final Intake intake;
    private final Axis axis;

    public ReverseIntake(Intake intake) {
        this.intake = intake;
        this.axis = null;
        addRequirements(this.intake);
    }

    public ReverseIntake(Intake intake, Axis axis) {
        this.intake = intake;
        this.axis = axis;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (this.axis != null) {
            this.intake.setSpeed(this.axis.getDeadzoned());
        } else {
            this.intake.setSpeed(INTAKE_REVERSE_SPEED);
        }
    }

    @Override
    public void execute() {
        if (this.axis != null) {
            this.intake.setSpeed(this.axis.getDeadzoned());
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.setSpeed(0);
    }
}
