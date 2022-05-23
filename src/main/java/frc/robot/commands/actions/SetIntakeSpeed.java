package frc.robot.commands.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SetIntakeSpeed extends CommandBase {
	private final Timer timer = new Timer();
	private final Intake intake;
	private final double speed;
	private final double duration;

	public SetIntakeSpeed(Intake intake, double speed, double duration) {
		this.intake = intake;
		this.speed = speed;
		this.duration = duration;
	}

	@Override
	public void initialize() {
		timer.start();
	}

	@Override
	public void execute() {
		if (timer.get() <= duration) intake.setSpeed(speed);
	}

	@Override
	public void end(boolean interrupted) {
		intake.setSpeed(0);
	}

	@Override
	public boolean isFinished() {
		return timer.get() >= duration;
	}
}
