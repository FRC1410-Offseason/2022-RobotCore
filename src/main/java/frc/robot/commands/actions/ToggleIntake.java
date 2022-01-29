package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


public class ToggleIntake extends CommandBase {
    private final Intake intake;

    public ToggleIntake(Intake intake) {
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        this.intake.toggle();
    }
}
