package frc.robotmap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;

public final class Tuning {

	public static final long DT = 5; // ms
	public static final long TIME_OFFSET = 3; // ms

	public static final double DRIVER_DEADZONE_VALUE = 0.05;
	public static final double OPERATOR_DEADZONE_VALUE = 0.05;

	public static final double SHOOTER_LEFT_KP = 0;
	public static final double SHOOTER_LEFT_KI = 0;
	public static final double SHOOTER_LEFT_KD = 0;
	public static final double SHOOTER_LEFT_KFF = 0;

	public static final double SHOOTER_RIGHT_KP = 0;
	public static final double SHOOTER_RIGHT_KI = 0;
	public static final double SHOOTER_RIGHT_KD = 0;
	public static final double SHOOTER_RIGHT_KFF = 0;

	public static final double DRIVETRAIN_MAX_VOLTAGE = 12;
	public static final double DRIVETRAIN_MAX_SPEED = 3.50;
	public static final double DRIVETRAIN_MAX_ACCEL = 3.80;
	public static final double DRIVETRAIN_MAX_CENTRIPETAL_ACCEL = 2.5;

	//Ramsete controller constants
	public static final double KB = 4.0;
	public static final double KZ = 0.7;

	public static final double KP_VEL = 5;

	public static final double MAX_VOLTAGE = 12;
	public static final double MAX_SPEED = 3.5; // m/s
	public static final double MAX_ACCEL = 4.8; // m/s^2
	public static final double MAX_CENTRIPETAL_ACCEL = 4.5; // m/s^2

	public static final double STATE_X = 0.000001;
	public static final double STATE_Y = 0.000001;
	public static final double STATE_THETA = 0.000001;
	public static final double STATE_LEFT_DIST = 0.000001;
	public static final double STATE_RIGHT_DIST = 0.000001;

	public static final double LOCAL_LEFT_DIST = 0.000001;
	public static final double LOCAL_RIGHT_DIST = 0.000001;
	public static final double LOCAL_THETA = 0.000001;

	public static final double VISION_X = 0.000001;
	public static final double VISION_Y = 0.000001;
	public static final double VISION_THETA = 0.000001;

	public static final Matrix<N7, N1> NOISE = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);
	public static final Matrix<N7, N1> NOISE_TEST = VecBuilder.fill(0, 0, 0, 0, 0, 0, 0);
}
