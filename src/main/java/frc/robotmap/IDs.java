package frc.robotmap;

public class IDs {
	public static final int[] LEFT_ENCODER_PORTS = new int[] {6, 7};
	public static final int[] RIGHT_ENCODER_PORTS = new int[] {8, 9};

	public static final int DRIVETRAIN_LEFT_FRONT_MOTOR_ID = 1;
	public static final int DRIVETRAIN_LEFT_BACK_MOTOR_ID = 2;
	public static final int DRIVETRAIN_RIGHT_FRONT_MOTOR_ID = 3;
	public static final int DRIVETRAIN_RIGHT_BACK_MOTOR_ID = 4;

	//SHOOTER
	public static final int SHOOTER_LEFT_MOTOR_ID = 11;
	public static final int SHOOTER_RIGHT_MOTOR_ID = 12;

	//STORAGE
	public static final int STORAGE_MOTOR_ID = 41;
	public static final int STORAGE_LINE_BREAK_PORT = 0; //To Do: Find out what the actual port on the roborio is

	//INTAKE 
	//To Do: Update with actual ports on the PCM
	public static final int INTAKE_FLIPPER_FWD = 6;
	public static final int INTAKE_FLIPPER_BCK = 7;
	public static final int INTAKE_MOTOR = 31;
	//Elevator
	//T0 D0: Update with motor ports
	public static final int ELEVATOR_FWD = 0;
	public static final int ELEVATOR_BCK = 1;
	public static final int ELEVATOR_LEFT_MOTOR_ID = 51;
	public final static int ELEVATOR_RIGHT_MOTOR_ID = 52;
	//WINCH
	//To Do: Update with actual ports on the PCM
	public static final int WINCH_FWD = 2;
	public static final int WINCH_BCK = 3;
	public static final int WINCH_LEFT_MOTOR_ID = 61;
	public static final int WINCH_RIGHT_MOTOR_ID = 62;
	//SHOOTER ARM
	public static final int SHOOTER_ARM_LOCK_FWD = 4;
	public static final int SHOOTER_ARM_LOCK_BCK = 5;
	public static final int SHOOTER_ARM_L_MOTOR = 21;
	public static final int SHOOTER_ARM_R_MOTOR = 22;
	public static final int PRESSURE_SENSOR = 0;

    //CONTROLLERS, DO NOT TOUCH
	public static final int DRIVER_CONTROLLER_PORT = 0;
	public static final int OPERATOR_CONTROLLER_PORT = 1;

	public enum AXIS_ID {
		LEFT_X,
		RIGHT_X,
		LEFT_Y,
		RIGHT_Y,
		LEFT_TRIGGER,
		RIGHT_TRIGGER
	}

	public enum BUTTON_ID {
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

		BUTTON_ID(int id) {
			this.id = id;
		}

		public int getId() {
			return id;
		}
	}

    public enum BUTTON_CONDITION {
		WHEN_PRESSED,
        WHEN_UNPRESSED,
        WHILE_HELD,
        TOGGLE_WHEN_PRESSED
	}

    public enum SCHEDULER_PRIORITY {
		NULL(0),
        LOW(1),
        MEDIUM(2),
        HIGH(3);

		private final int id;

		SCHEDULER_PRIORITY(int id) {
			this.id = id;
		}

		public int getId() {
			return id;
		}
	}
}
