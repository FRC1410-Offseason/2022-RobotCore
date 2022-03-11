package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RightTelescopingArm;


public class RunRightTAConstant extends CommandBase {

	private final RightTelescopingArm arm;
	private final double speed;

	public RunRightTAConstant(RightTelescopingArm arm, double speed) {
		this.arm = arm;
		this.speed = speed;
		addRequirements(this.arm);
	}

	@Override
	public void execute() {
		arm.set(speed);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		arm.set(0);
	}
}
