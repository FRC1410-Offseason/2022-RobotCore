package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.controllers.Axis;
import frc.robot.subsystems.Winch;


public class RunWinch extends CommandBase {

	private final Winch winch;
	private final Axis leftAxis;
	private final Axis rightAxis;

	public RunWinch(Winch winch, Axis leftAxis, Axis rightAxis) {
		this.winch = winch;
		this.leftAxis = leftAxis;
		this.rightAxis = rightAxis;
		addRequirements(winch);
	}

	@Override
	public void execute() {
		winch.runLeftWinch(leftAxis.getDeadzoned());
		winch.runRightWinch(rightAxis.getDeadzoned());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		winch.runRightWinch(0);
		winch.runLeftWinch(0);
	}
}
