package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


public class RetractIntake extends CommandBase {
    private final Intake intake;

    public RetractIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        this.intake.retract();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
