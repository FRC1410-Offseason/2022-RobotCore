package frc.robotmap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.numbers.N1;

public final class Constants {

	//TIMING
	public static final double DT50HZ = 20;
	public static final double DT200HZ = 5;
	public static final long TIME_OFFSET = 3;

	// ELEVATOR
	public static final double GEAR_RATIO = 10.61;
	public static final double ELEVATOR_MASS = 2.7; // kg, placeholder
	public static final double ELEVATOR_METERS_PER_REV = .0151;


	// SHOOTER ARM
	public static final double SHOOTER_ARM_RESTING_ANGLE = 19; // Degrees
	public static final double SHOOTER_ARM_INTAKE_ANGLE = 35; // Degrees
	public static final double SHOOTER_ARM_MAX_ANGLE = 51; // Degrees

	public static final int SHOOTER_ARM_GEARING = 125;
	public static final double SHOOTER_ARM_LENGTH = .8025; // Meters (nice)
	public static final double SHOOTER_ARM_MASS = 8.61; // Kg

	// DRIVETRAIN
	public static final double KS = .6803; // Position
	public static final double KV = 2.6629; // Velocity
	public static final double KA = .41365; // Acceleration
	public static final double KV_ANGULAR = 2.8; // Angular Velocity
	public static final double KA_ANGULAR = .2; // Angular Acceleration

	public static final double WHEEL_DIAMETER = .15; // meters
	public static final double ENCODER_CPR = 2048;
	public static final double ENCODER_EPR = 2048;
	public static final double GEARING = 11.998;
	public static final double METERS_PER_REVOLUTION = .4787787204061;
	public static final double TRACKWIDTH = .6907; // meters
	public static final double ENCODER_DISTANCE_PER_PULSE = (WHEEL_DIAMETER * Math.PI) / ENCODER_CPR;
	public static final double ENCODER_CONSTANT = (1 / GEARING) * (1 / ENCODER_EPR) * METERS_PER_REVOLUTION;

	public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACKWIDTH);

	public static final LinearSystem<N2, N2, N2> DRIVETRAIN_PLANT =
		LinearSystemId.identifyDrivetrainSystem(
			KV,
			KA,
			KV_ANGULAR,
			KA_ANGULAR);
	// LIMELIGHT
	public static final double UPPER_HUB_HEIGHT = 103;

	// STORAGE
	public static final Color BLUE_TARGET = new Color(new Color8Bit(100, 138, 216));
	public static final Color RED_TARGET = new Color(new Color8Bit(224, 69, 56));

	public static final double STORAGE_RUN_SPEED = 1;
	public static final double STORAGE_SHOOT_SPEED = 1;
	public static final double STORAGE_INTAKE_SPEED = 1;

	// INTAKE
	public static final double INTAKE_FORWARD_SPEED = 1;

	// SHOOTER
	public static final int SHOOTER_RPM_TOLERANCE = 100;

	// WINCH
	public static final double WINCH_LEFT_MOD = 0.7;

	// FEEDBACK
	public static final int LED_CONTROLLER_PORT = 0;
}
