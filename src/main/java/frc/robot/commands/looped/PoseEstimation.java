package frc.robot.commands.looped;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robotmap.Constants.*;

public class PoseEstimation extends CommandBase {
	static final NetworkTableInstance instance = NetworkTableInstance.getDefault();
	static final NetworkTable table = instance.getTable("Pose Estimation");
	static NetworkTableEntry x, y, theta, hz, navxangle, correctedNavxAngle, navxIsCalibrating; // Auto

	private final Drivetrain drivetrain;

	//TODO: Same here as with the drivetrain simulation command, we can probably condense a lot of this into function calls

	public PoseEstimation(Drivetrain drivetrain) {
		drivetrain.gyro.reset();

		x = table.getEntry("X");
		y = table.getEntry("Y");
		theta = table.getEntry("Heading");
		hz = table.getEntry("hz");
		navxangle = table.getEntry("navxangle");
		navxIsCalibrating = table.getEntry("NavX is Calibrating");
		x.setDouble(0);
		y.setDouble(0);
		theta.setDouble(0);
		hz.setDouble(0);
		navxangle.setDouble(0);
		navxIsCalibrating.setBoolean(false);

		this.drivetrain = drivetrain;
	}

	@Override
	public void execute() {
		drivetrain.leftEncoderPosition = ((drivetrain.leftLeader.getSelectedSensorPosition(0) +
				drivetrain.leftFollower.getSelectedSensorPosition(0)) * ENCODER_CONSTANT / 2);
		drivetrain.leftEncoderVelocity = ((drivetrain.leftLeader.getSelectedSensorVelocity(0) +
				drivetrain.leftFollower.getSelectedSensorVelocity(0)) * ENCODER_CONSTANT / 2 * 10);
		drivetrain.rightEncoderPosition = ((drivetrain.rightLeader.getSelectedSensorPosition(0) +
				drivetrain.rightFollower.getSelectedSensorPosition(0)) * ENCODER_CONSTANT / 2);
		drivetrain.rightEncoderVelocity = ((drivetrain.rightLeader.getSelectedSensorVelocity(0) +
				drivetrain.rightFollower.getSelectedSensorVelocity(0)) * ENCODER_CONSTANT / 2 * 10);
		// NEED TO CALCULATE ENCODER CALIBRATION CONSTANT

		drivetrain.poseEstimator.update(drivetrain.gyro.getRotation2d(), drivetrain.getWheelSpeeds(),
				drivetrain.leftEncoderPosition, drivetrain.rightEncoderPosition);

		x.setDouble(drivetrain.getPoseEstimation().getX());
		y.setDouble(drivetrain.getPoseEstimation().getY());
		theta.setDouble(drivetrain.getPoseEstimation().getRotation().getDegrees());
		navxangle.setDouble(drivetrain.gyro.getRotation2d().getDegrees());
		navxIsCalibrating.setBoolean(drivetrain.gyro.isCalibrating());
	}
}
