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

	// Feedforward for big math
	private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(INTAKE_KS, INTAKE_KV);

	// The physical limits of the mechanism
	private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(INTAKE_MAX_VEL, INTAKE_MAX_ACCEL);

	// The possible states of the intake
	private final TrapezoidProfile.State extendedState = new TrapezoidProfile.State(27, 0);
	private final TrapezoidProfile.State retractedState = new TrapezoidProfile.State(0, 0);

	// Used for control
	private TrapezoidProfile.State goal = null;
	private TrapezoidProfile.State lpr = null;


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

		// Default state is retracted
		goal = retractedState;
		lpr = new TrapezoidProfile.State();
	}

	@Override
	public void periodic() {
		// Get the current profile
		var profile = new TrapezoidProfile(constraints, goal, lpr);

		// Calculate the desired state based the profile
		lpr = profile.calculate(0.02);

		// Set the references of the PID controllers
		leftController.setReference(lpr.position, CANSparkMax.ControlType.kPosition, 0, feedforward.calculate(lpr.velocity));
		rightController.setReference(lpr.position, CANSparkMax.ControlType.kPosition, 0, feedforward.calculate(lpr.velocity));

	}

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
		if (goal == extendedState) {
			retract();
		} else {
			extend();
		}
	}

	/**
	 * Extend the intake
	 */
	public void extend() {
		if (goal != extendedState) {
			goal = extendedState;
		}
	}

	/**
	 * Retract the intake
	 */
	public void retract() {
		if (goal != retractedState) {
			goal = retractedState;
		};
	}
}
