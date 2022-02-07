package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NetworkTables;
import frc.robot.subsystems.Intake;


public class ExtendIntake extends CommandBase {

	private final Intake intake;

	public ExtendIntake(Intake intake) {
		this.intake = intake;
		// addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.extend();
		NetworkTables.setIntakeDeployed(true);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
