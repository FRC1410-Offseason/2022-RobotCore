package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;


public class RunIntakeWithButton extends CommandBase {

	private final Intake intake;
	private final Storage storage;

	public RunIntakeWithButton(Intake intake, Storage storage) {
		this.intake = intake;
		this.storage = storage;
		addRequirements(this.intake, this.storage);
	}

	@Override
	public void execute() {
		intake.setSpeed(1);
		storage.setIntaking(true);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		intake.setSpeed(0);
		storage.setIntaking(false);
	}
}
