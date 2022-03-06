package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.controllers.Axis;
import frc.robot.subsystems.Elevator;


public class RunElevator extends CommandBase {

	private final Elevator elevator;
	private final Axis leftAxis;
	private final Axis rightAxis;

	public RunElevator(Elevator elevator, Axis leftAxis, Axis rightAxis) {
		this.elevator = elevator;
		this.leftAxis = leftAxis;
		this.rightAxis = rightAxis;
		addRequirements(this.elevator);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double finalPower = leftAxis.getRaw() + (-1 * rightAxis.getRaw());
		elevator.set(finalPower);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		elevator.set(0);
	}
}
