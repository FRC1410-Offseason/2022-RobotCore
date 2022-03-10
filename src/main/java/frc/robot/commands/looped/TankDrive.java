package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.controllers.Axis;
import frc.robot.subsystems.Drivetrain;

import static frc.robotmap.Constants.*;

public class TankDrive extends CommandBase {
	private final Drivetrain drivetrain;
	private final Axis leftAxis;
	private final Axis rightAxis;

	public TankDrive(Drivetrain drivetrain, Axis leftAxis, Axis rightAxis) {
		this.drivetrain = drivetrain;
		this.leftAxis = leftAxis;
		this.rightAxis = rightAxis;
		// addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		drivetrain.tankDrive(leftAxis.getDeadzoned(), rightAxis.getDeadzoned(), false);

        System.out.println("Left Leader Encoder: " + drivetrain.leftLeader.getSelectedSensorPosition(0));
        System.out.println("Left Follower Encoder: " + drivetrain.leftFollower.getSelectedSensorPosition(0));
        System.out.println("Right Leader Encoder: " + drivetrain.rightLeader.getSelectedSensorPosition(0));
        System.out.println("Right Follower Encoder: " + drivetrain.rightFollower.getSelectedSensorPosition(0));
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
