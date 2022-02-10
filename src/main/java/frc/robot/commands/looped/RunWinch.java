package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.Axis;
import frc.robot.subsystems.Winch;


public class RunWinch extends CommandBase {

	private final Winch winch;
	private final Axis axis;

	public RunWinch(Winch winch, Axis axis) {
		this.winch = winch;
		this.axis = axis;
		addRequirements(winch);
	}

	@Override
	public void execute() {
		winch.unlock();
		winch.runWinch(axis.getDeadzoned());

		if (axis.getDeadzoned() == 0) {
			winch.runWinch(0);
			winch.lock();
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
