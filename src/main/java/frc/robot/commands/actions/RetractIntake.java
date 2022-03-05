package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeFlipper;


public class RetractIntake extends CommandBase {

	private final IntakeFlipper intakeFlipper;

	public RetractIntake(IntakeFlipper intakeFlipper) {
		this.intakeFlipper = intakeFlipper;
	}

	@Override
	public void initialize() {
		intakeFlipper.setDesiredPosition(false);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
