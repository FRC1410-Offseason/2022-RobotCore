package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeFlipper;


public class ExtendIntake extends CommandBase {

	private final IntakeFlipper intakeFlipper;

	public ExtendIntake(IntakeFlipper intakeFlipper) {
		this.intakeFlipper = intakeFlipper;
	}

	@Override
	public void initialize() {
		intakeFlipper.setDesiredPosition(true);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
