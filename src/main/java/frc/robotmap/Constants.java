// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robotmap;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	//ELEVATOR
	public static final double GEAR_RATIO = 10.61;
	public static final double ELEVATOR_MASS = 2.7; //kg, placeholder
	public static final double ELEVATOR_KV = 1; //Placeholder
	public static final double ELEVATOR_KA = 1; //Placeholder
	public static final double ELEVATOR_METERS_PER_REV = 0.0151;

	public static final double ELEVATOR_POS_CONFIDENCE = 0.0001; //Meters
	public static final double ELEVATOR_VEL_CONFIDENCE = 0.0001; //Meters per second
	public static final double ELEVATOR_ENC_CONFIDENCE = 0.0001; //Rotations

	public static final double ELEVATOR_POS_TOLERANCE = .001; //Meters
	public static final double ELEVATOR_VEL_TOLERANCE = .1; //Meters per second
	public static final double ELEVATOR_CTRL_TOLERANCE = 12; //Volts

	public static final double ELEVATOR_MAX_VEL = 1; //Meters per second
	public static final double ELEVATOR_MAX_ACCEL = 1; //Meters per second^2
	public static final double ELEVATOR_MAX_POS = 2; //Meters
	public static final double ELEVATOR_MIN_POS = 0; //Meters



	//SHOOTER ARM
	public static final double SHOOTER_ARM_KV = 0;
	public static final double SHOOTER_ARM_KA = 0;

	public static final int SHOOTER_ARM_GEARING = 125;
	public static final double SHOOTER_ARM_LENGTH = 0.69; //Meters (nice)
	public static final double SHOOTER_ARM_MASS = 10.21; //Kg
	public static final double SHOOTER_ARM_MIN_ROT = 0; //Deg
	public static final double SHOOTER_ARM_MAX_ROT = 60; //Deg

	public static final double SHOOTER_ARM_POS_CONFIDENCE = .0001;
	public static final double SHOOTER_ARM_VEL_CONFIDENCE = .0001;
	public static final double SHOOTER_ARM_ENC_CONFIDENCE = 1;

	public static final double SHOOTER_ARM_IS_FINISHED_THRESHOLD = 2; //Deg

	public static final double SHOOTER_ARM_POS_TOLERANCE = 0.1;
	public static final double SHOOTER_ARM_VEL_TOLERANCE = 0.01;
	public static final double SHOOTER_ARM_CTRL_TOLERANCE = 11;

//    public static final Matrix<N1, N1> SHOOTER_ARM_NOISE = VecBuilder.fill(0.01);

	public static final double SHOOTER_ARM_MAX_VOLTAGE = 12.;
	public static final double SHOOTER_ARM_MAX_VELOCITY = 40; // Deg per Second
	public static final double SHOOTER_ARM_MAX_ACCEL = 300; // Deg per Second squared

	public static final double DT = 0.02;

	//Position
	public static final double KS = 0.676;
	//Acceleration
	public static final double KV = 2.67;
	//Velocity
	public static final double KA = 0.34;
	//Angular Velocity
	public static final double KV_ANGULAR = 2.84;
	//Angular Acceleration
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

	//INTAKE
	public static final double INTAKE_FORWARD_SPEED = 1;
	public static final double INTAKE_REVERSE_SPEED = -1;
}
