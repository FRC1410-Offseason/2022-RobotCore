package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.framework.subsystem.SubsystemBase;
import frc.robot.util.storage.StorageState;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Constants.*;

public class Storage extends SubsystemBase {

	public enum ColorSensorStatus {
		ALLIANCE,
		NOT_ALLIANCE,
		EMPTY
	}

	private final WPI_VictorSPX motor = new WPI_VictorSPX(STORAGE_MOTOR_ID);

	private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

	private final ColorMatch colorMatch = new ColorMatch();

	private final DigitalInput lineBreak = new DigitalInput(STORAGE_LINE_BREAK_ID);

	private final StorageState currentState = new StorageState();

	private boolean lineBreakPrev = true;

	private boolean outtakeFlag = false;

	private final DriverStation.Alliance currentAlliance;

	public Storage(DriverStation.Alliance alliance) {
		currentAlliance = alliance;
		motor.configFactoryDefault();
		colorMatch.addColorMatch(RED_TARGET);
		colorMatch.addColorMatch(BLUE_TARGET);
	}

	@Override
	public void periodic() {
		//Falling edge
		if (lineBreakPrev && !lineBreak.get()) {
			motor.set(STORAGE_INDEX_SPEED);
			//If there is a ball in the first position, then move it to the second
			//and reset the first position to be ready for the next ball
			if (currentState.getSlot1().getBallPresent()) {
				currentState.setSlot2(true);
				currentState.resetSlot1();
			}
			//Rising Edge
		} else if (!lineBreakPrev && lineBreak.get()) {
			//Stop motor so we can read the color
			motor.set(0);
			//Read the color from the color sensor
			ColorMatchResult result = colorMatch.matchClosestColor(colorSensor.getColor());
			if (result.color.equals(RED_TARGET)) {
				if (currentAlliance == DriverStation.Alliance.Red) {
					//If the detected color is red, and we're on the red alliance, then the color is correct
					currentState.setSlot1(true);
				} else {
					//If the detected color is red, and we're on the blue alliance, then the color is incorrect
					currentState.setSlot1(false);
					//If the color is wrong then we need to set the outtake flag for the outtake handler
					outtakeFlag = true;
				}
			} else {
				if (currentAlliance == DriverStation.Alliance.Red) {
					//If the detected color is blue, and we're on the red alliance, then the color is incorrect
					currentState.setSlot1(false);
					//If the color is wrong then we need to set the outtake flag for the outtake handler
					outtakeFlag = true;
				} else {
					//If the detected color is blue, and we're on the blue alliance, then the color is correct
					currentState.setSlot1(true);
				}
			}
		}

		//Set lineBreakPrev to the current state of the line break in preparation for the next iteration
		lineBreakPrev = lineBreak.get();
	}

	public boolean getOuttakeFlag() {
		return outtakeFlag;
	}

	public StorageState getCurrentState() {
		return this.currentState;
	}

	public void runStorage(double power) {
		motor.set(power);
	}

//	public void setSlot1(boolean alliance) {
//		currentState.setSlot1(alliance);
//	}
//
//	public void setSlot2(boolean alliance) {
//		currentState.setSlot2(alliance);
//	}
//
//	public void resetSlot1() {
//		currentState.resetSlot1();
//	}
//
//	public void resetSlot2() {
//		currentState.resetSlot2();
//	}


}

