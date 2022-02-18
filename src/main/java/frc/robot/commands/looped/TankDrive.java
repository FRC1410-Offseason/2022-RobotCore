package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.controllers.Axis;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
	private final Drivetrain drivetrain;
	private final Axis leftAxis;
	private final Axis rightAxis;

	public TankDrive(Drivetrain drivetrain, Axis leftAxis, Axis rightAxis) {
		this.drivetrain = drivetrain;
		this.leftAxis = leftAxis;
		this.rightAxis = rightAxis;
		addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		drivetrain.tankDrive(leftAxis.getTeleopAntifriction(), rightAxis.getTeleopAntifriction());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.tankDriveVolts(0, 0);
	}
}