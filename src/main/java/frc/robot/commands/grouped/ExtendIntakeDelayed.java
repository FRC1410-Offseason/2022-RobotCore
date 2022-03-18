package frc.robot.commands.grouped;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.ExtendIntake;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeFlipper;
import static frc.robotmap.Tuning.*;

public class ExtendIntakeDelayed extends SequentialCommandGroup {

    public ExtendIntakeDelayed(IntakeFlipper intakeFlipper) {

        addCommands (
            new WaitCommand(INTAKE_DEPLOYMENT_DELAY),
            new ExtendIntake(intakeFlipper)
        );
    }   
}
