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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class Trajectories {
	private final Drivetrain drivetrain;

	private static TrajectoryConfig config = new TrajectoryConfig(DRIVETRAIN_MAX_SPEED, DRIVETRAIN_MAX_ACCEL)
		.setKinematics(DRIVE_KINEMATICS)
		.setReversed(false);

	private static TrajectoryConfig reverseConfig = new TrajectoryConfig(DRIVETRAIN_MAX_SPEED, DRIVETRAIN_MAX_ACCEL)
			.setKinematics(DRIVE_KINEMATICS)
			.setReversed(true);

	public final Trajectory straightline = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
		new Pose2d(2.5, 0, new Rotation2d(Units.degreesToRadians(0)))), config);
        
    public final Trajectory straightline2 = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
		new Pose2d(0, 2.5, new Rotation2d(Units.degreesToRadians(0)))), config);

	// Two Ball Auto
    public final Trajectory twoBallGet = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
		new Pose2d(2.5, 0, new Rotation2d(Units.degreesToRadians(0)))), config);

	public final Trajectory twoBallReturn = TrajectoryGenerator.generateTrajectory(List.of(
			new Pose2d(2.5, 0, new Rotation2d(Units.degreesToRadians(0))),
			new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0)))), reverseConfig);

	// Two Ball High
	public final Trajectory twoBall = TrajectoryGenerator.generateTrajectory(List.of(
			new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
			new Pose2d(2.3, 0, new Rotation2d(Units.degreesToRadians(0)))), config);

	public final Trajectory driveToShoot = TrajectoryGenerator.generateTrajectory(List.of(
			new Pose2d(1.2, 0, new Rotation2d(Units.degreesToRadians(0))),
			new Pose2d(2.0, 0, new Rotation2d(Units.degreesToRadians(0)))), config);

	// Three Ball Auto
	public final Trajectory threeBallGet = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
		new Pose2d(0.3, 0, new Rotation2d(Units.degreesToRadians(80))),
		new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(300)))), config); // TODO: Fill
	
	public final Trajectory threeBallReturn = TrajectoryGenerator.generateTrajectory(List.of(
		new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(300))), // TODO: Fill
		new Pose2d(0.3, 0, new Rotation2d(Units.degreesToRadians(80))),
		new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0)))), reverseConfig);
	
		
	public RamseteCommand straightlineCommand, straightlineCommand2, twoBallCommand, driveToShootCommand, twoBallGetCommand, twoBallReturnCommand, threeBallGetCommand, threeBallReturnCommand;

	public Trajectories(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
	}

	public RamseteCommand generateRamsete(Trajectory trajectory) {
		return new RamseteCommand(
			trajectory,
			drivetrain::getPoseEstimation,
			new RamseteController(KB, KZ),
			new SimpleMotorFeedforward(KS, KV, KA),
			DRIVE_KINEMATICS,
			drivetrain::getWheelSpeeds,
			new PIDController(KP_VEL, 0, 0, 10.0 / 1000),
			new PIDController(KP_VEL, 0, 0, 10.0 / 1000),
			drivetrain::tankDriveVolts
		);
	}

	public void setStartingAutonomousPose(Trajectory trajectory) {
		drivetrain.resetPoseEstimation(trajectory.getInitialPose());
	}

	public void generateAuto() {
		straightlineCommand = generateRamsete(straightline);
        straightlineCommand2 = generateRamsete(straightline2);
		twoBallCommand = generateRamsete(twoBall);
		driveToShootCommand = generateRamsete(driveToShoot);
        twoBallGetCommand = generateRamsete(twoBallGet);
		twoBallReturnCommand = generateRamsete(twoBallReturn);
		threeBallGetCommand = generateRamsete(threeBallGet);
		threeBallReturnCommand = generateRamsete(threeBallReturn);
	}
}
