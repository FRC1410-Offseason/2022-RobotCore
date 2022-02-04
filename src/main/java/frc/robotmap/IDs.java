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

    public enum ButtonId {
        A(1),
        B(2),
        X(3),
        Y(4),
        LEFT_BUMPER(5),
        RIGHT_BUMPER(6),
        LEFT_STICK_BUTTON(7),
        RIGHT_STICK_BUTTON(8);

        private final int id;

        ButtonId(int id) {
            this.id = id;
        }

        public int getId() {
            return id;
        }
    }

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final int STORAGE_MOTOR_ID = 1;
    //TODO: Find out what the actual port on the roborio is
    public static final int STORAGE_LINE_BREAK_PORT = 0;

    //INTAKE
    //TODO: Update with actual ports on the PCM
    public static final int INTAKE_FLIPPER_L_FWD = 0;
    public static final int INTAKE_FLIPPER_L_BCK = 1;
    public static final int INTAKE_FLIPPER_R_FWD = 2;
    public static final int INTAKE_FLIPPER_R_BCK = 3;

    public static final int INTAKE_MOTOR = 0;

    //WINCH
    //TODO: Update with actual ports on the PCM
    public static final int WINCH_L_FWD = 0;
    public static final int WINCH_L_BCK = 1;
    public static final int WINCH_R_FWD = 2;
    public static final int WINCH_R_BCK = 3;

    public static final int WINCH_LEFT_MOTOR_ID = 0;
    public static final int WINCH_RIGHT_MOTOR_ID = 0;

    public static final int PRESSURE_SENSOR = 0;
}
