package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.framework.subsystem.SubsystemBase;
import frc.robot.util.storage.StorageState;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Constants.*;

public class Storage extends SubsystemBase {

	private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
	private final NetworkTable table = instance.getTable("Storage");

	private final NetworkTableEntry slot1Ball = table.getEntry("Slot 1 Ball");
	private final NetworkTableEntry slot1Color = table.getEntry("Slot 1 Color");

	private final NetworkTableEntry slot2Ball = table.getEntry("Slot 2 Ball");
	private final NetworkTableEntry slot2Color = table.getEntry("Slot 2 Color");

	private final NetworkTableEntry lineBreakNt = table.getEntry("Line break");
	private final NetworkTableEntry color = table.getEntry("Color");

	private final NetworkTableEntry outtakeFlagNt = table.getEntry("Outtake");

	private final NetworkTableEntry encoderNt = table.getEntry("Shooter Arm Encoder");

	/**
	 * Used for the state objects, correct alliance color is passed into the subsystem on initialization
	 */
	public enum ColorSensorStatus {
		ALLIANCE,
		NOT_ALLIANCE,
		EMPTY
	}

	/**
	 * Motor that runs the storage (bag motor)
	 */
	private final WPI_TalonSRX motor = new WPI_TalonSRX(STORAGE_MOTOR_ID);

	/**
	 * Color sensor for reading the color of the cargo and updating state
	 */
	private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

	/**
	 * Used to match the color from the sensor that is closest to the known red and blue colors
	 */
	private final ColorMatch colorMatch = new ColorMatch();

	/**
	 * Line break sensor used to detect if ball is present or not
	 * From our testing, the color sensor is not able to consistently tell the difference between an empty storage and a red ball
	 * So this sensor is necessary to be completely certain what the know the state of the storage
	 */
	private final DigitalInput lineBreak = new DigitalInput(STORAGE_LINE_BREAK_ID);

	/**
	 * Used to keep track of the current state of the storage
	 */
	private final StorageState currentState = new StorageState();

	/**
	 * Used for detecting the rising/falling edge of the line break
	 */
	private boolean lineBreakPrev = true;

	private boolean intaking = false;

	private boolean manualControl = false;

	/**
	 * Signals to the rest of the code that there is a cargo of the wrong color somewhere in the storage
	 */
	private boolean outtakeFlag = false;

	/**
	 * Keeps track of what alliance we are on so that we know which color is the correct color
	 */
	private final DriverStation.Alliance currentAlliance;

	public Storage(DriverStation.Alliance alliance) {

		// Store the alliance that we are on
		currentAlliance = alliance;

		// Configure the motor
		motor.configFactoryDefault();

		// Add the two color targets to the color matcher
		colorMatch.addColorMatch(RED_TARGET);
		colorMatch.addColorMatch(BLUE_TARGET);
	}

	@Override
	public void periodic() {
		encoderNt.setDouble(motor.getSelectedSensorPosition());

//		if (intaking) {
//			runStorage(STORAGE_INTAKE_SPEED);
//		}

		slot1Ball.setBoolean(currentState.getSlot1().getBallPresent());
		slot2Ball.setBoolean(currentState.getSlot2().getBallPresent());

		lineBreakNt.setBoolean(lineBreak.get());
		outtakeFlagNt.setBoolean(outtakeFlag);

		var currentColor = colorMatch.matchClosestColor(colorSensor.getColor());
		if (currentColor.color.equals(RED_TARGET)) {
			color.setString("RED");
		} else if (currentColor.color.equals(BLUE_TARGET)) {
			color.setString("BLUE");
		} else {
			color.setString("NONE");
		}

		if (currentState.getSlot1().getColor() == ColorSensorStatus.ALLIANCE) {
			slot1Color.setString("ALLIANCE");
		} else {
			slot1Color.setString("NOT ALLIANCE");
		}

		if (currentState.getSlot2().getColor() == ColorSensorStatus.ALLIANCE) {
			slot2Color.setString("ALLIANCE");
		} else {
			slot2Color.setString("NOT ALLIANCE");
		}

		if (!manualControl) {
			// Falling edge
			if (lineBreakPrev && !lineBreak.get()) {
				motor.set(-1);
				// If there is a ball in the first position, then move it to the second
				// and reset the first position to be ready for the next ball
				if (currentState.getSlot1().getBallPresent()) {
					currentState.setSlot2(true);
					currentState.resetSlot1();
				}
				// Rising Edge
			} else if (!lineBreakPrev && lineBreak.get()) {
				// Stop motor so we can read the color
				motor.set(0);
				// Read the color from the color sensor
				ColorMatchResult result = colorMatch.matchClosestColor(colorSensor.getColor());
				if (result.color.equals(RED_TARGET)) {
					if (currentAlliance == DriverStation.Alliance.Red) {
						// If the detected color is red, and we're on the red alliance, then the color is correct
						currentState.setSlot1(true);
					} else {
						// If the detected color is red, and we're on the blue alliance, then the color is incorrect
						currentState.setSlot1(false);
						// If the color is wrong then we need to set the outtake flag for the outtake handler
						outtakeFlag = true;
					}
				} else {
					if (currentAlliance == DriverStation.Alliance.Red) {
						// If the detected color is blue, and we're on the red alliance, then the color is incorrect
						currentState.setSlot1(false);
						// If the color is wrong then we need to set the outtake flag for the outtake handler
						outtakeFlag = true;
					} else {
						// If the detected color is blue, and we're on the blue alliance, then the color is correct
						currentState.setSlot1(true);
					}
				}
			} else {
				if (intaking) {
					motor.set(STORAGE_INTAKE_SPEED);
				} else {
					motor.set(0);
				}
			}

		}

		// Set lineBreakPrev to the current state of the line break in preparation for the next iteration
		lineBreakPrev = lineBreak.get();
	}

	/**
	 * Get the current value of the outtake flag
	 * @return true if outtake is necessary, false otherwise
	 */
	public boolean getOuttakeFlag() {
		return outtakeFlag;
	}

	/**
	 * Used to reset the flag when outtaking has been done
	 * @param flag the boolean to set the flag to, should usually be false
	 */
	public void setOuttakeFlag(boolean flag) {
		outtakeFlag = flag;
	}

	public boolean isManualControl() {
		return manualControl;
	}

	public void setManualControl(boolean manualControl) {
		this.manualControl = manualControl;
	}

	public boolean isIntaking() {
		return intaking;
	}

	public void setIntaking(boolean intaking) {
		this.intaking = intaking;
	}

	public final WPI_TalonSRX getShooterArmMotor() {
		return motor;
	}

	/**
	 * Get the current state of the storage, used for determining the method of outtake
	 * @return a StorageState object that contains two StorageSlot objects
	 */
	public StorageState getCurrentState() {
		return this.currentState;
	}

	/**
	 * Run the storage motor at a certain power
	 * @param power a double from -1 to 1
	 */
	public void runStorage(double power) {
		motor.set(power);
	}

	public void resetOuttakeFlag() {
		outtakeFlag = false;
	}
}
