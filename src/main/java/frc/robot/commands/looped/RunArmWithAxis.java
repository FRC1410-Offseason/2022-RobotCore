package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.controllers.Axis;
import frc.robot.subsystems.ShooterArm;


public class RunArmWithAxis extends CommandBase {

	private final ShooterArm shooterArm;
	private final Axis axis;

	public RunArmWithAxis(ShooterArm shooterArm, Axis axis) {
		this.shooterArm = shooterArm;
		this.axis = axis;
		addRequirements(this.shooterArm);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		shooterArm.set(axis.getDeadzoned());
	}

	@Override
	public boolean isFinished() {
		// TODO: Make this return true when this Command no longer needs to run execute()
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
