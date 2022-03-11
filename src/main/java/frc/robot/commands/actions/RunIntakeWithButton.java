package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

import static frc.robotmap.Constants.*;
public class RunIntakeWithButton extends CommandBase {

	private final Intake intake;
	private final Storage storage;
    private final Shooter shooter;
    // private final Shooter shooter;

	public RunIntakeWithButton(Intake intake, Storage storage, Shooter shooter) {
		this.intake = intake;
		this.storage = storage;
        this.shooter = shooter;
        // this.shooter = shooter;
		addRequirements(this.intake, this.storage);
	}

	@Override
	public void execute() {
		intake.setSpeed(1);
		storage.runStorage(1);
        shooter.setSpeeds(-2000);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		intake.setSpeed(0);
		storage.runStorage(0);
        shooter.setSpeeds(0);
	}
}
