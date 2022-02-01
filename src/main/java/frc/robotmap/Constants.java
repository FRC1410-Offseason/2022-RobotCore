// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robotmap;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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

    public static final double SHOOTER_ARM_POS_TOLERANCE = 1;
    public static final double SHOOTER_ARM_VEL_TOLERANCE = 30;
    public static final double SHOOTER_ARM_CTRL_TOLERANCE = 12;

    public static final Matrix<N1, N1> SHOOTER_ARM_NOISE = VecBuilder.fill(0.01);

    public static final double SHOOTER_ARM_MAX_VOLTAGE = 12.;
    public static final double SHOOTER_ARM_MAX_VELOCITY = 30; // Deg per Second
    public static final double SHOOTER_ARM_MAX_ACCEL = 30; // Deg per Second squared

    public static final double DT = 0.02;
}
