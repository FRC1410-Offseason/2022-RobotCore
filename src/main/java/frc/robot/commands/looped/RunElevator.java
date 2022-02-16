package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.Axis;
import frc.robot.subsystems.Elevator;


public class RunElevator extends CommandBase {

	private final Elevator elevator;
	private final Axis axis;

	public RunElevator(Elevator elevator, Axis axis) {
		this.elevator = elevator;
		this.axis = axis;
		addRequirements(this.elevator);
	}

	@Override
	public void initialize() {
		elevator.set(0);
		elevator.lock();
	}

	@Override
	public void execute() {
		boolean shouldRun = !(Math.abs(axis.getDeadzoned()) < 0.1);
		if (shouldRun) {
			elevator.unlock();
			elevator.set(axis.getDeadzoned());
		} else {
			elevator.set(0);
			elevator.lock();
		}
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
