package frc.robot.util;

import static frc.robot.constants.Tuning.*;
import static frc.robot.constants.IDs.*;
import static frc.robot.constants.Constants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.subsystems.Drivetrain;

public class Trajectories {
    private Drivetrain drivetrain;
    private final TrajectoryConfig config;
    private final TrajectoryConfig reverseConfig;

    private CentripetalAccelerationConstraint centripetalAccelerationConstraint;
    private DifferentialDriveVoltageConstraint differentialDriveVoltageConstraint;

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
}
