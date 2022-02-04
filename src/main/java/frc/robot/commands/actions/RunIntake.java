package frc.robot.commands.actions;

import static frc.robotmap.Constants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.input.Axis;
import frc.robot.subsystems.Intake;


public class RunIntake extends CommandBase {
    private final Intake intake;
    private final Axis axis;

    public RunIntake(Intake intake) {
        this.intake = intake;
        this.axis = null;
        addRequirements(this.intake);
    }

    public RunIntake(Intake intake, Axis axis) {
        this.intake = intake;
        this.axis = axis;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (this.axis != null) {
            this.intake.setSpeed(this.axis.getDeadzoned());
        } else {
            this.intake.setSpeed(INTAKE_FORWARD_SPEED);
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
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.setSpeed(0);
    }
}
