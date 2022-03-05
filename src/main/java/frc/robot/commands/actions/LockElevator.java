package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;


public class LockElevator extends CommandBase {

	private final Elevator elevator;

	public LockElevator(Elevator elevator) {
		this.elevator = elevator;
		addRequirements(this.elevator);
	}

	@Override
	public void initialize() {
		elevator.lock();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
