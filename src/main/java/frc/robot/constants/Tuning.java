package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;

public final class Tuning {
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
}
