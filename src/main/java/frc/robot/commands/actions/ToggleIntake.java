package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeFlipper;


public class ToggleIntake extends CommandBase {

	private final IntakeFlipper intakeFlipper;

	public ToggleIntake(IntakeFlipper intakeFlipper) {
		this.intakeFlipper = intakeFlipper;
	}

	@Override
	public void initialize() {
		intakeFlipper.setDesiredPosition(!intakeFlipper.getDesiredPosition());
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
