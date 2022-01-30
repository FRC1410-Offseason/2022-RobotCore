// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robotmap;

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

    public static final double SHOOTER_ARM_POS_CONFIDENCE = 0.;
    public static final double SHOOTER_ARM_VEL_CONFIDENCE = 0.;
    public static final double SHOOTER_ARM_ENC_CONFIDENCE = 0.;

    public static final double SHOOTER_ARM_POS_TOLERANCE = 0.;
    public static final double SHOOTER_ARM_VEL_TOLERANCE = 0.;
    public static final double SHOOTER_ARM_CTRL_TOLERANCE = 0.;

    public static final double SHOOTER_ARM_MAX_VOLTAGE = 12.;
    public static final double SHOOTER_ARM_MAX_VELOCITY = 0.; // Rad per Second
    public static final double SHOOTER_ARM_MAX_ACCEL = 0.; // Rad per Second squared

    public static final double DT = 0.02;
}
