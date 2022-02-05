package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Winch;


public class ReleaseWinchBrake extends CommandBase {

	private final Winch winch;

	public ReleaseWinchBrake(Winch winch) {
		this.winch = winch;
		addRequirements(winch);
	}

	@Override
	public void initialize() {
		winch.unlock();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}