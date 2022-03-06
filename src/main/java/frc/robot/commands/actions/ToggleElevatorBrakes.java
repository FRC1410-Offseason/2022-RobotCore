package frc.robot.commands.actions;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;


public class ToggleElevatorBrakes extends CommandBase {

	private final Elevator elevator;

	public ToggleElevatorBrakes(Elevator elevator) {
		this.elevator = elevator;
		addRequirements(this.elevator);
	}

	@Override
	public void initialize() {
		elevator.setLock(!elevator.getLocked());
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
