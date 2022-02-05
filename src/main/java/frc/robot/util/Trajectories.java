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

public class Trajectories {
    private final Drivetrain drivetrain;
    
    private static final CentripetalAccelerationConstraint CentripetalAccelerationConstraint = 
        new CentripetalAccelerationConstraint(MAX_CENTRIPETAL_ACCEL);
    private static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(KS, KV, KA), DRIVE_KINEMATICS, MAX_VOLTAGE);

    private static final TrajectoryConfig config = new TrajectoryConfig(MAX_SPEED, MAX_ACCEL)
        .setKinematics(DRIVE_KINEMATICS)
        .addConstraint(CentripetalAccelerationConstraint)
        .addConstraint(autoVoltageConstraint)
        .setReversed(false);
    private static final TrajectoryConfig reverseConfig = new TrajectoryConfig(MAX_SPEED, MAX_ACCEL)
        .setKinematics(DRIVE_KINEMATICS)
        .addConstraint(CentripetalAccelerationConstraint)
        .addConstraint(autoVoltageConstraint)
        .setReversed(true);

    private static final Trajectory taxi = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(8.70, 6.50, new Rotation2d(Units.degreesToRadians(90))),
        new Pose2d(8.70, 7.60, new Rotation2d(Units.degreesToRadians(90)))), config);
    private static final Trajectory upperTarmacToUpperCargoShootTrajectory = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(8.75, 6.51, new Rotation2d(Units.degreesToRadians(91.3))),
        new Pose2d(8.835, 7.50, new Rotation2d(Units.degreesToRadians(80.56)))), config);
    private static final Trajectory upperCargoToUpperField = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(8.85, 7.63, new Rotation2d(Units.degreesToRadians(80.56))),
        new Pose2d(8.30, 6.90, new Rotation2d(Units.degreesToRadians(-15)))), reverseConfig);
    private static final Trajectory upperFieldToUpRightCargo = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(8.30, 6.90, new Rotation2d(Units.degreesToRadians(-15))),
        new Pose2d(10.97, 6.36, new Rotation2d(Units.degreesToRadians(0)))), config);
    private static final Trajectory upRightCargoToTerminalShoot = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(10.97, 6.36, new Rotation2d(Units.degreesToRadians(0))),
        new Pose2d(14.98, 6.95, new Rotation2d(Units.degreesToRadians(22.96)))), config);
    private static final Trajectory upperFieldToTerminalShoot = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(8.30, 6.30, new Rotation2d(Units.degreesToRadians(-15))),
        new Pose2d(15.10, 7.00, new Rotation2d(Units.degreesToRadians(22.9)))), config);

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
            drivetrain);
    }

    public Command Taxi() {
        RamseteCommand ramseteCommand = generateRamsete(taxi);
        drivetrain.resetPoseEstimation(taxi.getInitialPose());
        return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    }

    public Command UpperCargoShoot2() {
        RamseteCommand ramseteCommand = generateRamsete(upperTarmacToUpperCargoShootTrajectory);
        drivetrain.resetPoseEstimation(upperTarmacToUpperCargoShootTrajectory.getInitialPose());
        return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    }

    public Command Terminal3Cargo() {
        RamseteCommand ramseteCommand = generateRamsete(upperTarmacToUpperCargoShootTrajectory);
        RamseteCommand ramseteCommand2 = generateRamsete(upperCargoToUpperField);
        RamseteCommand ramseteCommand3 = generateRamsete(upperFieldToTerminalShoot);
        drivetrain.resetPoseEstimation(upperTarmacToUpperCargoShootTrajectory.getInitialPose());
        return ramseteCommand.andThen(
            () -> drivetrain.tankDriveVolts(0, 0)).andThen(
            new WaitCommand(1.7)).andThen(
            ramseteCommand2).andThen(
            ramseteCommand3).andThen(
            () -> drivetrain.tankDriveVolts(0, 0));
    }

    public Command UpRight3Cargo() {
        RamseteCommand ramseteCommand = generateRamsete(upperTarmacToUpperCargoShootTrajectory);
        RamseteCommand ramseteCommand2 = generateRamsete(upperCargoToUpperField);
        RamseteCommand ramseteCommand3 = generateRamsete(upperFieldToUpRightCargo);
        drivetrain.resetPoseEstimation(upperTarmacToUpperCargoShootTrajectory.getInitialPose());
        return ramseteCommand.andThen(
            () -> drivetrain.tankDriveVolts(0, 0)).andThen(
            new WaitCommand(1.7)).andThen(
            ramseteCommand2).andThen(
            ramseteCommand3).andThen(
            () -> drivetrain.tankDriveVolts(0, 0));
    }

    public Command UpRightTerminal4Cargo() {
        RamseteCommand ramseteCommand1 = generateRamsete(upperTarmacToUpperCargoShootTrajectory);
        RamseteCommand ramseteCommand2 = generateRamsete(upperCargoToUpperField);
        RamseteCommand ramseteCommand3 = generateRamsete(upperFieldToUpRightCargo);
        RamseteCommand ramseteCommand4 = generateRamsete(upRightCargoToTerminalShoot);
        drivetrain.resetPoseEstimation(upperTarmacToUpperCargoShootTrajectory.getInitialPose());
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

    public Command FiveCargo() {
        System.out.println("TOTAL TIME FOR TRAJECTORY 1 = " + upperTarmacToUpperCargoShootTrajectory.getTotalTimeSeconds());
        System.out.println("TOTAL TIME FOR TRAJECTORY 2 = " + upperCargoToUpperField.getTotalTimeSeconds());
        System.out.println("TOTAL TIME FOR TRAJECTORY 3 = " + upperFieldToUpRightCargo.getTotalTimeSeconds());
        System.out.println("TOTAL TIME FOR TRAJECTORY 4 = " + upRightCargoToTerminalShoot.getTotalTimeSeconds());
        RamseteCommand ramseteCommand1 = generateRamsete(upperTarmacToUpperCargoShootTrajectory);
        RamseteCommand ramseteCommand2 = generateRamsete(upperCargoToUpperField);
        RamseteCommand ramseteCommand3 = generateRamsete(upperFieldToUpRightCargo);
        RamseteCommand ramseteCommand4 = generateRamsete(upRightCargoToTerminalShoot);
        drivetrain.resetPoseEstimation(upperTarmacToUpperCargoShootTrajectory.getInitialPose());
        return ramseteCommand1.andThen(
            () -> drivetrain.tankDriveVolts(0, 0)).andThen(
            new WaitCommand(1.7)).andThen(
            // Lower storage to HIA 
            ramseteCommand2).andThen(
            ramseteCommand3).andThen(
            () -> drivetrain.tankDriveVolts(0, 0)).andThen(
            new WaitCommand(0.4)).andThen(
            ramseteCommand4).andThen(
            () -> drivetrain.tankDriveVolts(0, 0));
    }
}