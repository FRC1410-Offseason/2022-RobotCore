package frc.robotmap;

public class IDs {
	public static final int[] LEFT_ENCODER_PORTS = { 6, 7 };
	public static final int[] RIGHT_ENCODER_PORTS = { 8, 9 };

	public static final int DRIVETRAIN_LEFT_FRONT_MOTOR_ID = 1; // Talon FX / Falcon 500
	public static final int DRIVETRAIN_LEFT_BACK_MOTOR_ID = 2; // Talon FX / Falcon 500
	public static final int DRIVETRAIN_RIGHT_FRONT_MOTOR_ID = 3; // Talon FX / Falcon 500
	public static final int DRIVETRAIN_RIGHT_BACK_MOTOR_ID = 4; // Talon FX / Falcon 500

	//SHOOTER
	public static final int SHOOTER_LEFT_MOTOR_ID = 11; // Spark Max / NEO
	public static final int SHOOTER_RIGHT_MOTOR_ID = 12; // Spark Max / NEO

	// STORAGE
	public static final int STORAGE_MOTOR_ID = 41; // Talon SRX / Bag Motor
	public static final int STORAGE_LINE_BREAK_ID = 3;

	// INTAKE
	public static final int INTAKE_FLIPPER_LEFT = 32; // Spark Max / NEO 550
	public static final int INTAKE_FLIPPER_RIGHT = 33; // Spark Max / NEO 550
	public static final int INTAKE_MOTOR = 31; // Spark Max / NEO 550

	// ELEVATOR
	public static final int ELEVATOR_FWD = 2;
	public static final int ELEVATOR_BCK = 5;
	public static final int ELEVATOR_LEFT_MOTOR_ID = 51; // Spark Max / NEO 550
	public final static int ELEVATOR_RIGHT_MOTOR_ID = 52; // Spark Max / NEO 550

	// WINCH
	public static final int WINCH_FWD = 3;
	public static final int WINCH_BCK = 4;
	public static final int WINCH_LEFT_MOTOR_ID = 61; // Victor SPX / Bag?
	public static final int WINCH_RIGHT_MOTOR_ID = 62; // Victor SPX / Bag?
	public static final int WINCH_LEFT_LIMIT_SWITCH_ID = 1;
	public static final int WINCH_RIGHT_LIMIT_SWITCH_ID = 2;

	// SHOOTER ARM
	public static final int SHOOTER_ARM_L_MOTOR = 21; // Spark Max / NEO
	public static final int SHOOTER_ARM_R_MOTOR = 22; // Spark Max / NEO
	public static final int PRESSURE_SENSOR = 0;

    // CONTROLLERS, DO NOT TOUCH
	public static final int DRIVER_CONTROLLER_PORT = 0;
	public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int TEST_CONTROLLER_PORT = 2;

	public enum AxisID {
		LEFT_X,
		RIGHT_X,
		LEFT_Y,
		RIGHT_Y,
		LEFT_TRIGGER,
		RIGHT_TRIGGER
	}

	public enum ButtonID {
		A(1),
		B(2),
		X(3),
		Y(4),
		LEFT_BUMPER(5),
		RIGHT_BUMPER(6),
		LEFT_STICK_BUTTON(7),
		RIGHT_STICK_BUTTON(8),
        DPAD_UP(0),
        DPAD_DOWN(180),
        DPAD_LEFT(270),
        DPAD_RIGHT(90);

		private final int id;

		ButtonID(int id) {
			this.id = id;
		}

		public int getId() {
			return id;
		}
	}

    public enum ButtonCondition {
		WHEN_PRESSED,
        WHEN_UNPRESSED,
        WHILE_HELD,
        TOGGLE_WHEN_PRESSED
	}

    public enum SchedulerPriority {
		NULL(0),
        LOW(1),
        MEDIUM(2),
        HIGH(3);

		private final int id;

		SchedulerPriority(int id) {
			this.id = id;
		}

		public int getValue() {
			return id;
		}
	}
}
