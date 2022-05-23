package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.framework.subsystem.SubsystemBase;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Constants.*;

public class Winch extends SubsystemBase {

	/**
	 * Motors
	 */
	private final VictorSPX leftMotor = new VictorSPX(WINCH_LEFT_MOTOR_ID);
	private final VictorSPX rightMotor = new VictorSPX(WINCH_RIGHT_MOTOR_ID);

	public Winch() {
		// Configure motors
		leftMotor.configFactoryDefault();
		rightMotor.configFactoryDefault();

		leftMotor.setInverted(true);

		leftMotor.setNeutralMode(NeutralMode.Brake);
		rightMotor.setNeutralMode(NeutralMode.Brake);
	}

	public void runLeftWinch(double speed) {
		leftMotor.set(ControlMode.PercentOutput, speed);
	}

	public void runRightWinch(double speed) {
		rightMotor.set(ControlMode.PercentOutput, speed);
	}
}
