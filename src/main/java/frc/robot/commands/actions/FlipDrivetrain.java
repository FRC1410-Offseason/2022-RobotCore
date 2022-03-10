package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FlipDrivetrain extends CommandBase {

	private final Drivetrain drivetrain;

	public FlipDrivetrain(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
	}

	@Override
	public void initialize() {
		drivetrain.flip();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
