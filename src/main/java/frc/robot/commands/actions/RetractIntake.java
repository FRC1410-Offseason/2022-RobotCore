package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NetworkTables;
import frc.robot.subsystems.Intake;


public class RetractIntake extends CommandBase {

	private final Intake intake;

	public RetractIntake(Intake intake) {
		this.intake = intake;
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.retract();
		NetworkTables.setIntakeDeployed(false);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
