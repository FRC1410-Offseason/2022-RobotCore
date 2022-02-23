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

	public IntakeFlipper() {
		leftMotor.restoreFactoryDefaults();
		rightMotor.restoreFactoryDefaults();

		leftPID.setP(INTAKE_P);
		leftPID.setI(INTAKE_I);
		leftPID.setD(INTAKE_D);

		rightPID.setP(INTAKE_P);
		rightPID.setI(INTAKE_I);
		rightPID.setD(INTAKE_D);

		leftPID.setOutputRange(INTAKE_DOWN_POWERCAP, INTAKE_UP_POWERCAP);
		rightPID.setOutputRange(INTAKE_DOWN_POWERCAP, INTAKE_UP_POWERCAP);
	}
}

