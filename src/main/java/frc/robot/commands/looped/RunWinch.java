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
		if (axis.getDeadzoned() != 0) {
			winch.unlock();
			winch.runWinch(axis.getDeadzoned());
		} else {
			winch.runWinch(0);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		winch.runWinch(0);
	}
}
