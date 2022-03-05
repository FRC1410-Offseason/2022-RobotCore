package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.controllers.Axis;
import frc.robot.subsystems.IntakeFlipper;


public class RunIntakeFlipperWithAxis extends CommandBase {

	private final IntakeFlipper intakeFlipper;
	private final Axis axis;

	public RunIntakeFlipperWithAxis(IntakeFlipper intakeFlipper, Axis axis) {
		this.intakeFlipper = intakeFlipper;
		this.axis = axis;
//		addRequirements(intakeFlipper);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		System.out.println("Running");
		intakeFlipper.setSpeed(axis.getDeadzoned());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
