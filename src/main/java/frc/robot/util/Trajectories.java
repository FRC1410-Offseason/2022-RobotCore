package frc.robot.util;

import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

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
	public final Trajectory upperTarmacToUpperCargoShoot = TrajectoryGenerator.generateTrajectory(List.of(
			new Pose2d(8.75, 6.51, new Rotation2d(Units.degreesToRadians(91.3))),
			new Pose2d(8.835, 7.50, new Rotation2d(Units.degreesToRadians(80.56)))), config);
	public final Trajectory upperCargoToUpperField = TrajectoryGenerator.generateTrajectory(List.of(
			new Pose2d(8.85, 7.63, new Rotation2d(Units.degreesToRadians(80.56))),
			new Pose2d(8.30, 6.90, new Rotation2d(Units.degreesToRadians(-15)))), reverseConfig);
	public final Trajectory upperFieldToUpRightCargo = TrajectoryGenerator.generateTrajectory(List.of(
			new Pose2d(8.30, 6.90, new Rotation2d(Units.degreesToRadians(-15))),
			new Pose2d(10.97, 6.36, new Rotation2d(Units.degreesToRadians(0)))), config);
	public final Trajectory upRightCargoToTerminalShoot = TrajectoryGenerator.generateTrajectory(List.of(
			new Pose2d(10.97, 6.36, new Rotation2d(Units.degreesToRadians(0))),
			new Pose2d(14.98, 6.95, new Rotation2d(Units.degreesToRadians(22.96)))), config);
	public final Trajectory upperFieldToTerminalShoot = TrajectoryGenerator.generateTrajectory(List.of(
			new Pose2d(8.30, 6.30, new Rotation2d(Units.degreesToRadians(-15))),
			new Pose2d(15.10, 7.00, new Rotation2d(Units.degreesToRadians(22.9)))), config);

	public RamseteCommand upperTarmacToUpperCargoShootCommand;
	public RamseteCommand upperCargoToUpperFieldCommand;
	public RamseteCommand upperFieldToUpRightCargoCommand;
	public RamseteCommand upRightCargoToTerminalShootCommand;

	public Trajectories(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
		upperTarmacToUpperCargoShootCommand = generateRamsete(upperTarmacToUpperCargoShoot);
		upperCargoToUpperFieldCommand = generateRamsete(upperCargoToUpperField);
		upperFieldToUpRightCargoCommand = generateRamsete(upperFieldToUpRightCargo);
		upRightCargoToTerminalShootCommand = generateRamsete(upRightCargoToTerminalShoot);
	}

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

	public Command Taxi() {
		RamseteCommand ramseteCommand = generateRamsete(taxi);
		drivetrain.resetPoseEstimation(taxi.getInitialPose());
		return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
	}

	public Command UpperCargoShoot2() {
		RamseteCommand ramseteCommand = generateRamsete(upperTarmacToUpperCargoShoot);
		drivetrain.resetPoseEstimation(upperTarmacToUpperCargoShoot.getInitialPose());
		return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
	}

	public Command Terminal3Cargo() {
		RamseteCommand ramseteCommand = generateRamsete(upperTarmacToUpperCargoShoot);
		RamseteCommand ramseteCommand2 = generateRamsete(upperCargoToUpperField);
		RamseteCommand ramseteCommand3 = generateRamsete(upperFieldToTerminalShoot);
		drivetrain.resetPoseEstimation(upperTarmacToUpperCargoShoot.getInitialPose());
		return ramseteCommand.andThen(
			() -> drivetrain.tankDriveVolts(0, 0)).andThen(
			new WaitCommand(1.7)).andThen(
			ramseteCommand2).andThen(
			ramseteCommand3).andThen(
			() -> drivetrain.tankDriveVolts(0, 0));
	}

	public Command UpRight3Cargo() {
		RamseteCommand ramseteCommand = generateRamsete(upperTarmacToUpperCargoShoot);
		RamseteCommand ramseteCommand2 = generateRamsete(upperCargoToUpperField);
		RamseteCommand ramseteCommand3 = generateRamsete(upperFieldToUpRightCargo);
		drivetrain.resetPoseEstimation(upperTarmacToUpperCargoShoot.getInitialPose());
		return ramseteCommand.andThen(
			() -> drivetrain.tankDriveVolts(0, 0)).andThen(
			new WaitCommand(1.7)).andThen(
			ramseteCommand2).andThen(
			ramseteCommand3).andThen(
			() -> drivetrain.tankDriveVolts(0, 0));
	}

	public Command UpRightTerminal4Cargo() {
		RamseteCommand ramseteCommand1 = generateRamsete(upperTarmacToUpperCargoShoot);
		RamseteCommand ramseteCommand2 = generateRamsete(upperCargoToUpperField);
		RamseteCommand ramseteCommand3 = generateRamsete(upperFieldToUpRightCargo);
		RamseteCommand ramseteCommand4 = generateRamsete(upRightCargoToTerminalShoot);
		drivetrain.resetPoseEstimation(upperTarmacToUpperCargoShoot.getInitialPose());
		return ramseteCommand1.andThen(
			() -> drivetrain.tankDriveVolts(0, 0)).andThen(
			new WaitCommand(1.7)).andThen(
			ramseteCommand2).andThen(
			ramseteCommand3).andThen(
			() -> drivetrain.tankDriveVolts(0, 0)).andThen(
			new WaitCommand(0.4)).andThen(
			ramseteCommand4).andThen(
			() -> drivetrain.tankDriveVolts(0, 0));
	}

	public Command FiveCargoPoseSet() {
		return new RunCommand(() -> drivetrain.resetPoseEstimation(upperTarmacToUpperCargoShoot.getInitialPose()));
	}

	public Command FiveCargo() {
		RamseteCommand upperTarmacToUpperCargoShootCommand = generateRamsete(upperTarmacToUpperCargoShoot);
		RamseteCommand upperCargoToUpperFieldCommand = generateRamsete(upperCargoToUpperField);
		RamseteCommand upperFieldToUpRightCargoCommand = generateRamsete(upperFieldToUpRightCargo);
		RamseteCommand upRightCargoToTerminalShootCommand = generateRamsete(upRightCargoToTerminalShoot);
		drivetrain.resetPoseEstimation(upperTarmacToUpperCargoShoot.getInitialPose());

		return 
		upperTarmacToUpperCargoShootCommand.andThen(
		() -> drivetrain.tankDriveVolts(0, 0)).andThen(
		new WaitCommand(1.7)).andThen(
		upperCargoToUpperFieldCommand).andThen(
		upperFieldToUpRightCargoCommand).andThen(
		() -> drivetrain.tankDriveVolts(0, 0)).andThen(
		new WaitCommand(0.4)).andThen(
		upRightCargoToTerminalShootCommand)/*.andThen(
		() -> drivetrain.tankDriveVolts(0, 0))*/;
	}
}
