package frc.robot.commands.actions;

import static frc.robotmap.Constants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;


public class RunIntake extends CommandBase {
    private final Intake intake;

    public RunIntake(Intake intake) {
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void initialize() {
        this.intake.setSpeed(INTAKE_FORWARD_SPEED);
    }
}
