package frc.robotmap;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class Constants {
    public static final double KS = 0.676;
    public static final double KV = 2.67;
    public static final double KA = 0.34;
    public static final double KV_ANGULAR = 2.84;
    public static final double KA_ANGULAR = 0.216;

    public static final double WHEEL_DIAMETER = 0.15; // meters
    public static final double ENCODER_CPR = 2048;
    public static final double ENCODER_EPR = 2048;
    public static final double GEARING = 11.998;
    public static final double METERS_PER_REVOLUTION = 0.4787787204061;
    public static final double TRACKWIDTH = 0.615; // meters
    public static final double ENCODER_DISTANCE_PER_PULSE = (WHEEL_DIAMETER * Math.PI) / (double) ENCODER_CPR;
    public static final double ENCODER_CONSTANT = (1 / GEARING) * (1 / ENCODER_EPR) * METERS_PER_REVOLUTION;

    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACKWIDTH);

    public static final LinearSystem<N2, N2, N2> DRIVETRAIN_PLANT =
            LinearSystemId.identifyDrivetrainSystem(
                    KV,
                    KA,
                    KV_ANGULAR,
                    KA_ANGULAR);

    public static final Color BLUE_TARGET = new Color(new Color8Bit(100, 138, 216));
    public static final Color RED_TARGET = new Color(new Color8Bit(224, 69, 56));

    public static final double STORAGE_INDEX_SPEED = 0;
}
