package frc.robotmap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.numbers.N1;

public final class Constants {

	//Elevator
	public static final double GEAR_RATIO = 10.61;
	//SHOOTER ARM
	public static final double SHOOTER_ARM_KV = 0;
	public static final double SHOOTER_ARM_KA = 0;

	public static final int SHOOTER_ARM_GEARING = 125;
	public static final double SHOOTER_ARM_LENGTH = 0.8025; //Meters (nice)
	public static final double SHOOTER_ARM_MASS = 8.61; //Kg
	public static final double SHOOTER_ARM_MIN_ROT = 0; //Deg
	public static final double SHOOTER_ARM_MAX_ROT = 60; //Deg

	public static final double SHOOTER_ARM_POS_CONFIDENCE = .0001;
	public static final double SHOOTER_ARM_VEL_CONFIDENCE = .0001;
	public static final double SHOOTER_ARM_ENC_CONFIDENCE = 1;

	public static final double SHOOTER_ARM_IS_FINISHED_THRESHOLD = 0.5; //Deg

	public static final double SHOOTER_ARM_POS_TOLERANCE = 0.1;
	public static final double SHOOTER_ARM_VEL_TOLERANCE = 0.01;
	public static final double SHOOTER_ARM_CTRL_TOLERANCE = 11;

	public static final Matrix<N1, N1> SHOOTER_ARM_NOISE = VecBuilder.fill(0.01);

	public static final double SHOOTER_ARM_MAX_VOLTAGE = 12.0;
	public static final double SHOOTER_ARM_MAX_VELOCITY = 170; // Deg per Second
	public static final double SHOOTER_ARM_MAX_ACCEL = 360; // Deg per Second squared
	//DRIVETRAIN
	public static final double DT = 0.02;

	public static final double KS = 0.676; //Position
	public static final double KV = 2.67; //Velocity
	public static final double KA = 0.34; //Acceleration
	public static final double KV_ANGULAR = 2.84; //Angular Velocity
	public static final double KA_ANGULAR = 0.216; //Angular Acceleration

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
	//SHOOTER MATH
	public static final double SIGNIFICANT_DROP_DETECTION_THSHLD = 0.1;
	public static final long LOWEST_EXPIRATION_TIME_MS = 150; //Maximum time between negative peaks
	public static final int REGRESSION_DEGREE = 4;
	public static final int REGRESSION_STEPS = 5000;
	public static final int REGRESSION_STEPS_PER_CYCLE = 200;
	public static final int REGRESSION_BUFFERSIZE = 20;
	public static final double REGRESSION_ALPHA = 0.000006;
	public static final double REGRESSION_STEPSIZE = 0.00000000005;
	public static final double REGRESSION_NOISE = 200;
	public static final double SHOOTER_WHEEL_RADIUS = 0.0762; //M
	public static final double SHOOTER_BALL_MASS = 0.270; //KG
	public static final double SHOOTER_I = 1;

	//STORAGE
	public static final Color BLUE_TARGET = new Color(new Color8Bit(100, 138, 216));
	public static final Color RED_TARGET = new Color(new Color8Bit(224, 69, 56));

	public static final double STORAGE_INDEX_SPEED = 1;
	public static final double STORAGE_RUN_SPEED = 1;
	public static final double STORAGE_REVERSE_SPEED = -1;

	//INTAKE
	public static final double INTAKE_FORWARD_SPEED = 1;
	public static final double INTAKE_REVERSE_SPEED = -1;
}
