package frc.robotmap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;

public final class Tuning {

    public static final double DRIVER_DEADZONE_VALUE = 0.05;
    public static final double OPERATOR_DEADZONE_VALUE = 0.05;

    public static final double MAX_VOLTAGE = 12;
    public static final double MAX_SPEED = 3.50;
    public static final double MAX_ACCEL = 3.80;
    public static final double MAX_CENTRIPETAL_ACCEL = 2.5;

    public static final double KB = 4.0;
    public static final double KZ = 0.7;

    public static final double KP_VEL = 5;

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
