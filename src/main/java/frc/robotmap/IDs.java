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
    
    public static final int DRIVETRAIN_LEFT_FRONT_MOTOR_ID = 1;
    public static final int DRIVETRAIN_LEFT_BACK_MOTOR_ID = 2;
    public static final int DRIVETRAIN_RIGHT_FRONT_MOTOR_ID = 3;
    public static final int DRIVETRAIN_RIGHT_BACK_MOTOR_ID = 4;

    public static final int SHOOTER_LEFT_MOTOR_ID = 10;
    public static final int SHOOTER_RIGHT_MOTOR_ID = 11;

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

    //SHOOTER ARM
    public static final int SHOOTER_ARM_LOCK_FWD = 6;
    public static final int SHOOTER_ARM_LOCK_BCK = 7;

    public static final int SHOOTER_ARM_L_MOTOR = 10;
    public static final int SHOOTER_ARM_R_MOTOR = 11;
  
    public static final int PRESSURE_SENSOR = 0;
}
