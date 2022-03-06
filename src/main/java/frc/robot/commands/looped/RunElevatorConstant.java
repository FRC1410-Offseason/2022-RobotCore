package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;


public class RunElevatorConstant extends CommandBase {

	private final Elevator elevator;
	private final double speed;

	public RunElevatorConstant(Elevator elevator, double speed) {
		this.elevator = elevator;
		this.speed = speed;
		addRequirements(this.elevator);
	}

	@Override
	public void initialize() {
		elevator.set(0);
//		elevator.lock();
	}

	@Override
	public void execute() {
//		elevator.unlock();
		elevator.set(speed);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		elevator.set(0);
//		elevator.lock();
	}
}
