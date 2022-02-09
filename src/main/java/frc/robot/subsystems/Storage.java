package frc.robot.subsystems;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;

public class Storage extends SubsystemBase {
    //Motor that runs the storage
	private final WPI_VictorSPX motor = new WPI_VictorSPX(STORAGE_MOTOR_ID);
    //Color sensor
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    //Line break sensor
    private final DigitalInput lineBreak = new DigitalInput(STORAGE_LINE_BREAK_PORT);
    private ColorSensorStatus currentColor = null;
    private BallStatus ballStatus = null;
	private OuttakeStatus outtakeStatus = null;

    public enum ColorSensorStatus {
        ALLIANCE,
        NOT_ALLIANCE
    }

    public enum BallStatus {
        INDEXING,
        INDEXED
    }

	public enum OuttakeStatus {
		OUTTAKING,
		NOT_OUTTAKING
	}

    public Storage() {
		motor.configFactoryDefault();
    }

    /**
     * Runs the storage at a given speed
     * @param power double from -1 to 1
     */
    public void runStorage(double power) {
        motor.set(power);
    }

    /**
     * Returns the current value that the line break is reporting
     * @return true if there is no cargo, false if there is
     */
    public boolean getLineBreak() {
        return lineBreak.get();
    }

	/**
	 * Returns the current color that the color sensor is reporting
	 * @return color object of color sensor
	 */
	public Color getColorSensor() {
		return colorSensor.getColor();
	}

	/**
	 * Returns the current state of the color sensor
	 * @return either RED or BLUE
	 */
	public ColorSensorStatus getCurrentColor() {
		return currentColor;
	}

	/**
	 * Returns the current state of the ball
	 * @return either INDEXING or INDEXED
	 */
	public BallStatus getBallStatus() {
		return ballStatus;
	}

	/**
	 * Returns the current state of outtaking
	 * @return either OUTTAKING or NOT_OUTTAKING
	 */
	public OuttakeStatus getOuttakeStatus() {
		return outtakeStatus;
	}

	/**
	 * Sets the current state of the color sensor
	 * @param currentColor either ALLIANCE or NOT_ALLIANCE
	 */
	public void setCurrentColor(ColorSensorStatus currentColor) {
		this.currentColor = currentColor;
	}

	/**
	 * Sets the current state of the ball
	 * @param ballStatus either INDEXING or INDEXED
	 */
	public void setBallStatus(BallStatus ballStatus) {
		this.ballStatus = ballStatus;
	}

	/**
	 * Sets the current state of outtaking
	 * @param outtakeStatus either OUTTAKING or NOT_OUTTAKING
	 */
	public void setOuttakeStatus(OuttakeStatus outtakeStatus) {
		this.outtakeStatus = outtakeStatus;
	}
}

