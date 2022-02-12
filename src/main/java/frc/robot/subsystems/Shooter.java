package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.NetworkTables;
import frc.robot.framework.subsystem.SubsystemBase;
import frc.robot.util.PolynomialRegression;
import frc.robotmap.Constants;
import it.unimi.dsi.fastutil.doubles.DoubleArrayList;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

import static frc.robotmap.IDs.SHOOTER_LEFT_MOTOR_ID;
import static frc.robotmap.IDs.SHOOTER_RIGHT_MOTOR_ID;
import static frc.robotmap.Tuning.*;

public class Shooter extends SubsystemBase {

	//Declare Motors
	private final CANSparkMax leftMotor = new CANSparkMax(SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);

	//Grab Encoders From Motors
	private final RelativeEncoder leftEncoder = leftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
	private final RelativeEncoder rightEncoder = rightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

	//Grab PID Controllers
	private final SparkMaxPIDController leftController = leftMotor.getPIDController();
	private final SparkMaxPIDController rightController = rightMotor.getPIDController();

	private double target = 0;
	private double pTarget = 0;
	private double lowestRPM = -1;
	private double lowestTime = 0;
	private final PolynomialRegression invR = new PolynomialRegression(Constants.REGRESSION_DEGREE, Constants.REGRESSION_BUFFERSIZE);
	private final PolynomialRegression pInvR = new PolynomialRegression(Constants.REGRESSION_DEGREE, 1);
	private boolean runningRegression = false;
	private int regressionStepCount = 0;
	private final List<Double> xValues = new DoubleArrayList();
	private final List<Double> xValuesNormalized = new DoubleArrayList();
	private final List<Double> yValues = new DoubleArrayList();
	private final List<DoubleConsumer> onShot = new ArrayList<>();

	public List<DoubleConsumer> getOnShotCallbacks() {
	    return onShot;
    }
    
	public void addOnShotCallback(DoubleConsumer callback) {
	    onShot.add(callback);
    }
    
	public boolean removeOnShotCallback(DoubleConsumer callback) {
	    return onShot.remove(callback);
    }
    
	public void clearOnShotCallbacks() {
	    onShot.clear();
    }

	/**
	 * Creates a new Shooter.
	 */
	public Shooter() {
		//Configure Motors
		leftMotor.restoreFactoryDefaults();
		rightMotor.restoreFactoryDefaults();
		leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
		rightMotor.setInverted(true);

		//Configure PID controller outputs
		leftController.setOutputRange(-1, 1);
		rightController.setOutputRange(-1, 1);

		//Set PID loops to default values from the tuning file
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
	}
	/**
	 * Use the regression to find a target RPM based on a target velocity in m/s.
	 * @param vel
	 * @return RPM
	 */
	public double targetRPM(double vel) {
		return 30.0 * pInvR.f(Math.pow(vel,2)
			* ((Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2) * Constants.SHOOTER_BALL_MASS) + Constants.SHOOTER_I)
			/ (Constants.SHOOTER_I * Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2))) / Math.PI;
	}

	@Override
	public void periodic() {
		double rpm = Math.abs((getRightVel() + getLeftVel()) * 0.5); //Average RPM
		if (rpm < (Math.min(Math.abs(pTarget), Math.abs(target)) * (1 - Constants.SIGNIFICANT_DROP_DETECTION_THSHLD))) {
			if (rpm < lowestRPM || lowestRPM < 0) {
				lowestRPM = rpm;
				lowestTime = System.currentTimeMillis();
			}
		} else if (Math.abs((rpm - Math.abs(target)) / target) <= (Constants.SIGNIFICANT_DROP_DETECTION_THSHLD * 0.5)) {
			pTarget = target;
		}

		if (lowestRPM >= 0) {
			if (System.currentTimeMillis() - lowestTime > Constants.LOWEST_EXPIRATION_TIME_MS) {
				//Throw out and use the current results - we're done dropping
				double omegaI = Math.abs(pTarget) * Math.PI / 30.0;
				double omegaF = lowestRPM * Math.PI / 30.0;
				double exitVelocity = Math.sqrt(Constants.SHOOTER_I / Constants.SHOOTER_BALL_MASS * (Math.pow(omegaI, 2) - Math.pow(omegaF, 2)));

				for (DoubleConsumer callback : onShot) {
					callback.accept(exitVelocity);
				}

				double xValue = ((Math.pow(omegaI, 2) - Math.pow(omegaF, 2)) * ((Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2)
					* Constants.SHOOTER_BALL_MASS) + Constants.SHOOTER_I))
					/ (Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2) * Constants.SHOOTER_BALL_MASS);

				if (xValue < 0 || xValue > Math.pow(omegaI, 2)) {
					throw new IllegalStateException("Incorrect calculation of xvalue stuff. Value: " + xValue);
				}

				xValues.add(xValue);
				yValues.add(omegaI);

				runningRegression = true;
				//Run normalization process
				xValuesNormalized.clear();
				invR.reset();
				double maximum = 0;

				for (double value : xValues) {
					if (value > maximum) maximum = value;
				}

				for (double value : xValues) {
					xValuesNormalized.add(value / maximum); //Do normalization
				}

				//Change lowestRPM stuff
				lowestRPM = -1;
				lowestTime = 0;
			}
		}

		if (runningRegression) {
			for (int i = 0; i < Constants.REGRESSION_STEPS_PER_CYCLE; i++) {
				invR.gradStep(
						Constants.REGRESSION_ALPHA, xValuesNormalized, yValues,
						Constants.REGRESSION_STEPSIZE, Constants.REGRESSION_NOISE
				);
			}

			regressionStepCount += Constants.REGRESSION_STEPS_PER_CYCLE;
			if (regressionStepCount >= Constants.REGRESSION_STEPS) {
				invR.setLowestInBuffer();
				runningRegression = false;

				//Copy parameter values over to PinvR
				System.arraycopy(invR.getParameters(), 0, pInvR.getParameters(), 0, invR.getParameters().length);
			}
		}

		NetworkTables.setLowestRPM(lowestRPM);
	}

	public void setLeftPID(double p, double i, double d) {
		setLeftPID(p, i, d, 0);
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

	public void setRightPID(double p, double i, double d) {
		setRightPID(p, i, d, 0);
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

}
