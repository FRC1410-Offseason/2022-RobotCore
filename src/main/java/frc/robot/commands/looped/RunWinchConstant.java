package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Winch;


public class RunWinchConstant extends CommandBase {

	private final Winch winch;
	private final double speed;

	public RunWinchConstant(Winch winch, double speed) {
		this.winch = winch;
		this.speed = speed;
		addRequirements(winch);
	}

	@Override
	public void execute() {
		winch.unlock();
		winch.runWinch(speed);
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
