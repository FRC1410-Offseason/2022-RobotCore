package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.NetworkTables;
import frc.robot.framework.subsystem.SubsystemBase;
import frc.robot.util.GradientDescentOptimized;
import frc.robot.util.PolynomialRegression;
import it.unimi.dsi.fastutil.doubles.DoubleArrayList;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

import static frc.robotmap.Constants.*;
import static frc.robotmap.IDs.SHOOTER_LEFT_MOTOR_ID;
import static frc.robotmap.IDs.SHOOTER_RIGHT_MOTOR_ID;
import static frc.robotmap.Tuning.*;

public class Shooter extends SubsystemBase {

	private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
	private final NetworkTable table = instance.getTable("Shooter");

	private final NetworkTableEntry leftRPM = table.getEntry("Left RPM");
	private final NetworkTableEntry rightRPM = table.getEntry("Right RPM");
	private final NetworkTableEntry targetRPM = table.getEntry("Target RPM");

	// Declare Motors
	private final CANSparkMax leftMotor = new CANSparkMax(SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);

	// Grab Encoders From Motors
	private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
	private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

	// Grab PID Controllers
	private final SparkMaxPIDController leftController = leftMotor.getPIDController();
	private final SparkMaxPIDController rightController = rightMotor.getPIDController();

	private double target = 0;

	private boolean outtakeQueued = false;

	/**
	 * Creates a new Shooter.
	 */
	public Shooter() {
		// Configure Motors
		leftMotor.restoreFactoryDefaults();
		rightMotor.restoreFactoryDefaults();
		leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rightMotor.setInverted(true);

		// Configure PID controller outputs
		leftController.setOutputRange(-1, 1);
		rightController.setOutputRange(-1, 1);

		// Set PID loops to default values from the tuning file
		setLeftPID(
			SHOOTER_LEFT_KP,
			SHOOTER_LEFT_KI,
			SHOOTER_LEFT_KD,
			SHOOTER_LEFT_KFF
		);
		setRightPID(
			SHOOTER_RIGHT_KP,
			SHOOTER_RIGHT_KI,
			SHOOTER_RIGHT_KD,
			SHOOTER_RIGHT_KFF
		);

		leftRPM.setDouble(0);
		rightRPM.setDouble(0);
		targetRPM.setDouble(0);
  }

	public void queueOuttake() {
		outtakeQueued = true;
	}

	public boolean isOuttakeQueued() {
		return outtakeQueued;
	}

	@Override
	public void periodic() {
		leftRPM.setDouble(leftEncoder.getVelocity());
		rightRPM.setDouble(rightEncoder.getVelocity());
		targetRPM.setDouble(target);
	}

	/**
	 * Set the PID constants for the left controller
	 *
	 * @param p  proportional gain
	 * @param i  integral gain
	 * @param d  derivative gain
	 * @param ff feed-forward gain
	 */
	public void setLeftPID(double p, double i, double d, double ff) {
		leftController.setP(p);
		leftController.setI(i);
		leftController.setD(d);
		leftController.setFF(ff);
	}

	/**
	 * Set PID Constants for the right controller
	 *
	 * @param p  proportional gain
	 * @param i  integral gain
	 * @param d  derivative gain
	 * @param ff feed-forward gain
	 */
	public void setRightPID(double p, double i, double d, double ff) {
		rightController.setP(p);
		rightController.setI(i);
		rightController.setD(d);
		rightController.setFF(ff);
	}


	/**
	 * Sets the target velocities for the two NEOs in revolutions per minute
	 *
	 * @param rpm speed of motors in revolutions per minute
	 */
	public void setSpeeds(double rpm) {
		leftController.setIAccum(0);
		rightController.setIAccum(0);

		target = rpm;

		leftController.setReference(target, CANSparkMax.ControlType.kVelocity);
		rightController.setReference(target, CANSparkMax.ControlType.kVelocity);
	}

	/**
	 * Returns the current reference speed for the shooter mechanism
	 *
	 * @return The target speed (RPM)
	 */
	public double getSpeed() {
		return target;
	}

	/**
	 * Returns the velocity of the left motor specifically
	 *
	 * @return The velocity of the left motor (RPM)
	 */
	public double getLeftVel() {
		return leftEncoder.getVelocity();
	}

	/**
	 * Returns the velocity of the right motor specifically
	 *
	 * @return The velocity of the right motor (RPM)
	 */
	public double getRightVel() {
		return rightEncoder.getVelocity();
	}

	/**
	 * Whether the flywheels are close enough to their target velocity to start shooting
	 * @return boolean True -> We are good to shoot, False -> Can't start shooting yet
	 */
	public boolean isAtTarget() {
		return Math.abs((getLeftVel() + getRightVel()) / 2 - target) < SHOOTER_RPM_TOLERANCE;
	}

}
