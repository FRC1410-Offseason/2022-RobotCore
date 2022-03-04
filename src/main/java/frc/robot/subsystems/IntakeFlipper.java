package frc.robot.subsystems;


import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.framework.subsystem.SubsystemBase;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.*;

public class IntakeFlipper extends SubsystemBase {

	private final CANSparkMax leftMotor = new CANSparkMax(INTAKE_FLIPPER_LEFT, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(INTAKE_FLIPPER_RIGHT, MotorType.kBrushless);

	private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
	private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

	private final SparkMaxPIDController leftPID = leftMotor.getPIDController();
	private final SparkMaxPIDController rightPID = rightMotor.getPIDController();

	// False will be the retracted state
	private boolean desiredPosition = false;

	public IntakeFlipper() {
		leftMotor.restoreFactoryDefaults();
		rightMotor.restoreFactoryDefaults();

		rightMotor.setInverted(true);

		leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		leftPID.setP(INTAKE_P);
		leftPID.setI(INTAKE_I);
		leftPID.setD(INTAKE_D);

		rightPID.setP(INTAKE_P);
		rightPID.setI(INTAKE_I);
		rightPID.setD(INTAKE_D);

		leftPID.setOutputRange(INTAKE_DOWN_POWERCAP, INTAKE_UP_POWERCAP);
		rightPID.setOutputRange(INTAKE_DOWN_POWERCAP, INTAKE_UP_POWERCAP);

		resetEncoders(0);
	}

	public void setSpeed(double speed) {
		leftMotor.set(speed);
		rightMotor.set(speed);
	}

	public double getEncoderPosition() {
		return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
	}

	public void resetEncoders() {
		leftEncoder.setPosition(0);
		rightEncoder.setPosition(0);
	}

	public void resetEncoders(double value) {
		leftEncoder.setPosition(value);
		rightEncoder.setPosition(value);
	}

	public void setDesiredPosition(boolean value) {
		desiredPosition = value;
	}

	public boolean getDesiredPosition() {
		return desiredPosition;
	}

	public void setPIDSpeed(double target) {
		leftPID.setReference(target, CANSparkMax.ControlType.kPosition);
		rightPID.setReference(target, CANSparkMax.ControlType.kPosition);
	}
}

