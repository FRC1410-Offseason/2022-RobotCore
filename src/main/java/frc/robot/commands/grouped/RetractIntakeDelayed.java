package frc.robot.commands.grouped;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.actions.RetractIntake;
import frc.robot.subsystems.IntakeFlipper;
import static frc.robotmap.Tuning.*;

public class RetractIntakeDelayed extends SequentialCommandGroup {

	public RetractIntakeDelayed(IntakeFlipper intakeFlipper) {

		addCommands (
				new WaitCommand(INTAKE_DEPLOYMENT_DELAY),
				new RetractIntake(intakeFlipper)
		);
	}
}
