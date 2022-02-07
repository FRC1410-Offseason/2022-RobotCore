package frc.robot.util;

import frc.robot.subsystems.Drivetrain;
import static frc.robotmap.Tuning.*;
import static frc.robotmap.Constants.*;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class Trajectories {
	private final Drivetrain drivetrain;

	private static final CentripetalAccelerationConstraint CentripetalAccelerationConstraint =
		new CentripetalAccelerationConstraint(DRIVETRAIN_MAX_CENTRIPETAL_ACCEL);

	private static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
		new SimpleMotorFeedforward(KS, KV, KA), DRIVE_KINEMATICS, DRIVETRAIN_MAX_VOLTAGE);

	private static final TrajectoryConfig config = new TrajectoryConfig(DRIVETRAIN_MAX_SPEED, DRIVETRAIN_MAX_ACCEL)
		.setKinematics(DRIVE_KINEMATICS)
		.addConstraint(CentripetalAccelerationConstraint)
		.addConstraint(autoVoltageConstraint)
		.setReversed(false);
	private static final TrajectoryConfig reverseConfig = new TrajectoryConfig(DRIVETRAIN_MAX_SPEED, DRIVETRAIN_MAX_ACCEL)
		.setKinematics(DRIVE_KINEMATICS)
		.addConstraint(CentripetalAccelerationConstraint)
		.addConstraint(autoVoltageConstraint)
		.setReversed(true);

	public final Trajectory taxi = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(8.70, 6.50, new Rotation2d(Units.degreesToRadians(90))),
		new Pose2d(8.70, 7.60, new Rotation2d(Units.degreesToRadians(90)))), config);
	public final Trajectory upperTarmacToUpperCargoShot = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(8.75, 6.51, new Rotation2d(Units.degreesToRadians(91.3))),
		new Pose2d(8.835, 7.50, new Rotation2d(Units.degreesToRadians(80.56)))), config);
	public final Trajectory upperCargoToUpperField = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(8.85, 7.63, new Rotation2d(Units.degreesToRadians(80.56))),
		new Pose2d(8.30, 6.90, new Rotation2d(Units.degreesToRadians(-15)))), reverseConfig);
	public final Trajectory upperFieldToUpRightCargo = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(8.30, 6.90, new Rotation2d(Units.degreesToRadians(-15))),
		new Pose2d(10.97, 6.36, new Rotation2d(Units.degreesToRadians(0)))), config);
	public final Trajectory upRightCargoToTerminalShot = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(10.97, 6.36, new Rotation2d(Units.degreesToRadians(0))),
		new Pose2d(14.98, 6.95, new Rotation2d(Units.degreesToRadians(22.96)))), config);
	public final Trajectory upperFieldToTerminalShot = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(8.30, 6.30, new Rotation2d(Units.degreesToRadians(-15))),
		new Pose2d(15.10, 7.00, new Rotation2d(Units.degreesToRadians(22.9)))), config);

	public RamseteCommand taxiCommand, upperTarmacToUpperCargoShotCommand, upperCargoToUpperFieldCommand;
	public RamseteCommand upperFieldToTerminalShotCommand, upperFieldToUpRightCargoCommand, upRightCargoToTerminalShotCommand;

	public Trajectories(Drivetrain drivetrain) {this.drivetrain = drivetrain;}

	public RamseteCommand generateRamsete(Trajectory trajectory) {
		return new RamseteCommand(
			trajectory,
			drivetrain::getPoseEstimation,
			new RamseteController(KB, KZ),
			new SimpleMotorFeedforward(KS, KV, KA),
			DRIVE_KINEMATICS,
			drivetrain::getWheelSpeeds,
			new PIDController(KP_VEL, 0, 0, 0.005),
			new PIDController(KP_VEL, 0, 0, 0.005),
			drivetrain::tankDriveVolts,
			drivetrain
		);
	}

	public void setStartingAutonomousPose() {drivetrain.resetPoseEstimation(upperTarmacToUpperCargoShot.getInitialPose());}

	public void generateAuto() {
		taxiCommand = generateRamsete(taxi);
		upperTarmacToUpperCargoShotCommand = generateRamsete(upperTarmacToUpperCargoShot);
		upperCargoToUpperFieldCommand = generateRamsete(upperCargoToUpperField);
		upperFieldToUpRightCargoCommand = generateRamsete(upperFieldToUpRightCargo);
		upperFieldToTerminalShotCommand = generateRamsete(upperFieldToTerminalShot);
		upRightCargoToTerminalShotCommand = generateRamsete(upRightCargoToTerminalShot);
	}
}
