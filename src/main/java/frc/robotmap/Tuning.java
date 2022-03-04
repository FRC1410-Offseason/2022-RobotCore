package frc.robotmap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;

public final class Tuning {

	public static final double DRIVER_DEADZONE_VALUE = 0.12;
	public static final double OPERATOR_DEADZONE_VALUE = 0.12;
  	public static final double TEST_DEADZONE_VALUE = 0.12;

  	// SHOOTER
	public static final double SHOOTER_LEFT_KP = 0.000140;
	public static final double SHOOTER_LEFT_KI = 5e-8;
	public static final double SHOOTER_LEFT_KD = 0;
	public static final double SHOOTER_LEFT_KFF = 0.000170;

	public static final double SHOOTER_RIGHT_KP = 0.000140;
	public static final double SHOOTER_RIGHT_KI = 5e-8;
	public static final double SHOOTER_RIGHT_KD = 0;
	public static final double SHOOTER_RIGHT_KFF = 0.000170;

	public static final double SHOOT_STORAGE_DURATION = 3.0;

	// LIMELIGHT
	public static final double LIMELIGHT_ANGLE_KP = 0.12; // 3.24V at 27deg
	public static final double LIMELIGHT_ANGLE_KI = 0;
	public static final double LIMELIGHT_ANGLE_KD = 0.012;
	// ^ Last tested was 0.024, so this is 1.5x higher. If you can crank D enough, maybe increase P

	// Ramsete controller constants
	public static final double KB = 2.0;
	public static final double KZ = 0.7;

	public static final double KP_VEL = 4.071;
	// From SysId, but may be better to use a lower value

	// public static final double DRIVETRAIN_MAX_VOLTAGE = 12;
	public static final double DRIVETRAIN_MAX_SPEED = 3.5; // m/s
	public static final double DRIVETRAIN_MAX_ACCEL = 1; // m/s^2
	// public static final double DRIVETRAIN_MAX_CENTRIPETAL_ACCEL = 3.2; // m/s^2

	// These values trust vision the most, then physics (characterization etc) and the measurements (gyro, encoders) equally
	public static final double STATE_X = 0.01;
	public static final double STATE_Y = 0.01;
	public static final double STATE_THETA = 0.01;
	public static final double STATE_LEFT_DIST = 0.01;
	public static final double STATE_RIGHT_DIST = 0.01;

	public static final double LOCAL_LEFT_DIST = 0.01;
	public static final double LOCAL_RIGHT_DIST = 0.01;
	public static final double LOCAL_THETA = 0.001;

	public static final double VISION_X = 0.0001;
	public static final double VISION_Y = 0.0001;
	public static final double VISION_THETA = 0.0001;

	public static final Matrix<N7, N1> NOISE = VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005);

	// INTAKE
	public static final double INTAKE_P = 1;
	public static final double INTAKE_I = 0;
	public static final double INTAKE_D = 0;

	public static final double INTAKE_DOWN_POWERCAP = -0.5;
	public static final double INTAKE_UP_POWERCAP = 0.5;

	public static final double INTAKE_IS_FINISHED = 0.5; // Rotations
	public static final double INTAKE_UP_POSITION = 0; // Rotations
	public static final double INTAKE_DOWN_POSITION = 12; // Rotations. From testing, real number will be different

	// SHOOTER ARM
	public static final double SA_P = 0.027; // Probably needs to be slightly more aggressive, but that's for later
	public static final double SA_I = 0;
	public static final double SA_D = 0;

	public static final double SA_P_UP = 0.045;
	public static final double SA_I_UP = 0;
	public static final double SA_D_UP = 0;

	public static final double SHOOTER_ARM_IS_FINISHED = 1; // Degree

	// SHOOTER
	public static final double SHOOT_STORAGE_DURATION = 2.0;
	public static final double AUTONOMOUS_SHOOTING_RPM = 1855;
}
