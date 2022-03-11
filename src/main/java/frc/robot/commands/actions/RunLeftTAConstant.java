package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LeftTelescopingArm;


public class RunLeftTAConstant extends CommandBase {

	private final LeftTelescopingArm arm;
	private final double speed;

	public RunLeftTAConstant(LeftTelescopingArm arm, double speed) {
		this.arm = arm;
		this.speed = speed;
		addRequirements(this.arm);
	}

	@Override
	public void initialize() {
		arm.set(0);
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
