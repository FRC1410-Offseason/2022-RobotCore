package frc.robotmap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;

public final class Tuning {
    public static final long DT = 5; // ms
    public static final long TIME_OFFSET = 3; // ms

    public static final double DRIVER_DEADZONE_VALUE = 0.12;
    public static final double OPERATOR_DEADZONE_VALUE = 0.12;

    public static final double SHOOTER_LEFT_kP = 0;
    public static final double SHOOTER_LEFT_kI = 0;
    public static final double SHOOTER_LEFT_kD = 0;
    public static final double SHOOTER_LEFT_kFF = 0;

    public static final double SHOOTER_RIGHT_kP = 0;
    public static final double SHOOTER_RIGHT_kI = 0;
    public static final double SHOOTER_RIGHT_kD = 0;
    public static final double SHOOTER_RIGHT_kFF = 0;
  
    public static final double DRIVETRAIN_MAX_VOLTAGE = 12;
    public static final double DRIVETRAIN_MAX_SPEED = 3.50;
    public static final double DRIVETRAIN_MAX_ACCEL = 4.80;
    public static final double DRIVETRAIN_MAX_CENTRIPETAL_ACCEL = 4.5;

    //Ramsete controller constants
    public static final double KB = 2.0;
    public static final double KZ = 0.7;

    public static final double KP_VEL = 10.0;

    public static final double MAX_VOLTAGE = 12;
	public static final double MAX_SPEED = 3.5; // m/s
    public static final double MAX_ACCEL = 4.8; // m/s^2
    public static final double MAX_CENTRIPETAL_ACCEL = 4.5; // m/s^2

    // Larger stddev means the pose estimator cares less about changes in that info, and will assume changes in the  those high stddev
    // measurements. Lower stddev means changes in these measurements will be "taken more seriously"
    // State (Characterization Based Feedforward) Standard Deviations
    public static final double STATE_X = 0.001;
    public static final double STATE_Y = 0.001;
    public static final double STATE_THETA = 0.001;
    public static final double STATE_LEFT_DIST = 0.005;
    public static final double STATE_RIGHT_DIST = 0.005;
    // Local Measurement (Encoders, Gyro) Standard Deviations
    public static final double LOCAL_LEFT_DIST = 0.005;
    public static final double LOCAL_RIGHT_DIST = 0.005;
    public static final double LOCAL_THETA = 0.001;
    // Vision Measurement (Limelight) Standard Deviations
    public static final double VISION_X = 1;
    public static final double VISION_Y = 1;
    public static final double VISION_THETA = 1;

	public static final Matrix<N7, N1> NOISE = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);
    // Stddevs for measurement noise: x and y:0.001 m|heading:0.001 rad|l and r velocity:0.1 m/s|l and r position:0.005 m
}
