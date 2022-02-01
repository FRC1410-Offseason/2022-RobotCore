package frc.robot.util;

import static frc.robotmap.Constants.*;
import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.*;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

public class Trajectories {
    private final Drivetrain drivetrain;
    private final TrajectoryConfig config;
    private final TrajectoryConfig reverseConfig;

    private final CentripetalAccelerationConstraint centripetalAccelerationConstraint;
    private final DifferentialDriveVoltageConstraint differentialDriveVoltageConstraint;

    public Trajectories(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCEL);

        differentialDriveVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(KS, KV, KA), DRIVE_KINEMATICS, MAX_VOLTAGE
        );

        config = new TrajectoryConfig(MAX_SPEED, MAX_ACCEL)
                .setKinematics(DRIVE_KINEMATICS)
                .addConstraint(centripetalAccelerationConstraint)
                .addConstraint(differentialDriveVoltageConstraint)
                .setReversed(false);

        reverseConfig = new TrajectoryConfig(MAX_SPEED, MAX_ACCEL)
                .setKinematics(DRIVE_KINEMATICS)
                .addConstraint(centripetalAccelerationConstraint)
                .addConstraint(differentialDriveVoltageConstraint)
                .setReversed(true);
    }

    public RamseteCommand generateRamsete(Trajectory trajectory) {
        return new RamseteCommand(
                trajectory,
                drivetrain::getPose,
                new RamseteController(KB, KZ),
                new SimpleMotorFeedforward(KS, KV, KA),
                DRIVE_KINEMATICS,
                drivetrain::getWheelSpeeds,
                new PIDController(KP_VEL, 0, 0),
                new PIDController(KP_VEL, 0, 0),
                drivetrain::tankDriveVolts,
                drivetrain
        );
    }

    public Command simpleTest() {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(90))),
                new Pose2d(0, 1, new Rotation2d(Units.degreesToRadians(90)))
        ), config);

        drivetrain.resetPoseEstimator(traj.getInitialPose());
        return generateRamsete(traj);
    }

    public Command test() {
        Trajectory forward = TrajectoryGenerator.generateTrajectory(List.of(
                        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(0, 2, new Rotation2d(Units.degreesToRadians(90)))),
                config);
        Trajectory reverse = TrajectoryGenerator.generateTrajectory(List.of(
                        new Pose2d(0, 2, new Rotation2d(Units.degreesToRadians(90))),
                        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(90)))),
                reverseConfig);

        RamseteCommand ramseteCommand = generateRamsete(forward);
        RamseteCommand ramseteCommand2 = generateRamsete(reverse);
        drivetrain.resetPoseEstimator(forward.getInitialPose());

        return ramseteCommand.andThen(
                ramseteCommand2).andThen(
                () -> drivetrain.tankDriveVolts(0, 0));
    }

    public Command test2() {
        Trajectory forward = TrajectoryGenerator.generateTrajectory(List.of(
                        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(2, 0, new Rotation2d(Units.degreesToRadians(0)))),
                config);
        Trajectory reverse = TrajectoryGenerator.generateTrajectory(List.of(
                        new Pose2d(2, 0, new Rotation2d(Units.degreesToRadians(0))),
                        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0)))),
                reverseConfig);

        RamseteCommand ramseteCommand = generateRamsete(forward);
        RamseteCommand ramseteCommand2 = generateRamsete(reverse);
        drivetrain.resetPoseEstimator(forward.getInitialPose());

        return ramseteCommand.andThen(
                ramseteCommand2).andThen(
                () -> drivetrain.tankDriveVolts(0, 0));
    }

    public Command fiveBall() {
        Trajectory first = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(9.07, 6.42, new Rotation2d(Units.degreesToRadians(67))),
                new Pose2d(9.05, 7.63, new Rotation2d(Units.degreesToRadians(110)))), config);
        Trajectory second = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(9.05, 7.63, new Rotation2d(Units.degreesToRadians(110))),
                new Pose2d(9.07, 6.12, new Rotation2d(Units.degreesToRadians(67)))), reverseConfig);
        Trajectory third = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(9.07, 6.12, new Rotation2d(Units.degreesToRadians(72))),
                new Pose2d(11.15, 6.20, new Rotation2d(Units.degreesToRadians(35.5)))), config);
        Trajectory fourth = TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(11.15, 6.20, new Rotation2d(Units.degreesToRadians(35.5))),
                new Pose2d(15.10, 7.02, new Rotation2d(Units.degreesToRadians(22.9)))), config);

        RamseteCommand ramseteCommand1 = generateRamsete(first);
        RamseteCommand ramseteCommand2 = generateRamsete(second);
        RamseteCommand ramseteCommand3 = generateRamsete(third);
        RamseteCommand ramseteCommand4 = generateRamsete(fourth);
        drivetrain.resetPoseEstimator(first.getInitialPose());

        return ramseteCommand1.andThen(
                ramseteCommand2).andThen(
                () -> drivetrain.tankDriveVolts(0, 0)).andThen(
                new WaitCommand(1.5)).andThen(
                ramseteCommand3).andThen(
                () -> drivetrain.tankDriveVolts(0, 0)).andThen(
                new WaitCommand(2.2)).andThen(
                ramseteCommand4).andThen(
                () -> drivetrain.tankDriveVolts(0, 0)).andThen(
                new WaitCommand(2.2));
    }


}
