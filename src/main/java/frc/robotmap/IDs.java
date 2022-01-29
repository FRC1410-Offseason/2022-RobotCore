package frc.robotmap;

public class IDs {
    public enum AXIS_ID {
        LEFT_X,
        RIGHT_X,
        LEFT_Y,
        RIGHT_Y,
        LEFT_TRIGGER,
        RIGHT_TRIGGER
    }

    public static final int A_BUTTON_ID = 1;
    public static final int B_BUTTON_ID = 2;
    public static final int X_BUTTON_ID = 3;
    public static final int Y_BUTTON_ID = 4;
    public static final int LEFT_BUMPER_ID = 5;
    public static final int RIGHT_BUMPER_ID = 6;
    public static final int LEFT_STICK_BUTTON_ID = 9;
    public static final int RIGHT_STICK_BUTTON_ID = 10;

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    //INTAKE
    //TODO: Update with actual ports on the PCM
    public static final int INTAKE_FLIPPER_L_FWD = 0;
    public static final int INTAKE_FLIPPER_L_BCK = 1;
    public static final int INTAKE_FLIPPER_R_FWD = 2;
    public static final int INTAKE_FLIPPER_R_BCK = 3;

    public static final int INTAKE_MOTOR = 0;
}
