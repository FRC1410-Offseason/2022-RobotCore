package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.framework.subsystem.SubsystemBase;

import static frc.robotmap.Constants.*;
import static frc.robotmap.Tuning.*;
import static frc.robotmap.IDs.*;

public class Intake extends SubsystemBase {

	// Flipper motors
	private final CANSparkMax flipperLeft = new CANSparkMax(INTAKE_FLIPPER_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax flipperRight = new CANSparkMax(INTAKE_FLIPPER_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);

	// PID Controller Objects
	private final SparkMaxPIDController leftController = flipperLeft.getPIDController();
	private final SparkMaxPIDController rightController = flipperRight.getPIDController();

	// Motor that runs the intake
	private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

	private boolean extended = false;


	public Intake() {
		// Config motors
		intakeMotor.restoreFactoryDefaults();

		flipperLeft.restoreFactoryDefaults();
		flipperRight.restoreFactoryDefaults();

		// Set up PID controllers
		leftController.setP(INTAKE_P);
		leftController.setI(INTAKE_I);
		leftController.setD(INTAKE_D);

		rightController.setP(INTAKE_P);
		rightController.setI(INTAKE_I);
		rightController.setD(INTAKE_D);
	}

	@Override
	public void periodic() {}

	/**
	 * Returns the speed of the intake
	 *
	 * @return Speed of motor
	 */
	public double getSpeed() {
		return intakeMotor.get();
	}

	/**
	 * Set the speed of the intake
	 *
	 * @param speed Speed from -1 to 1
	 */
	public void setSpeed(double speed) {
		intakeMotor.set(speed);
	}

	public void setRawFlipperSpeed(double speed) {
		flipperRight.set(speed);
		flipperLeft.set(speed);
	}

	public void setRawVoltage(double voltage) {
		flipperLeft.setVoltage(voltage);
		flipperRight.setVoltage(voltage);
	}

	/**
	 * Toggle the position of the intake
	 */
	public void toggle() {
		if (extended) {
			retract();
		} else {
			extend();
		}
	}

	/**
	 * Extend the intake
	 */
	public void extend() {
		if (!extended) {

		}
	}

	/**
	 * Retract the intake
	 */
	public void retract() {
		if (extended) {
			retract();
		}
	}
}
