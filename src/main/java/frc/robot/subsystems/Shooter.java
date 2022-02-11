package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.NetworkTables;
import frc.robot.framework.subsystem.SubsystemBase;
import frc.robot.util.PolynomialRegression;
import frc.robotmap.Constants;

import static frc.robotmap.IDs.SHOOTER_LEFT_MOTOR_ID;
import static frc.robotmap.IDs.SHOOTER_RIGHT_MOTOR_ID;
import static frc.robotmap.Tuning.*;

import java.util.List;
import java.util.ArrayList;
import java.util.function.Consumer;

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
	private double ptarget = 0;
	private double lowestRPM = -1;
	private double lowestTime = 0;
	private PolynomialRegression invR = new PolynomialRegression(Constants.REGRESSION_DEGREE, Constants.REGRESSION_BUFFERSIZE);
	private PolynomialRegression PinvR = new PolynomialRegression(Constants.REGRESSION_DEGREE, 1);
	private boolean runningRegression = false;
	private int regressionStepcount = 0;
	private final List<Double> xValues = new ArrayList<Double>();
	private final List<Double> xValuesNormalized = new ArrayList<Double>();
	private final List<Double> yValues = new ArrayList<Double>();
	private ArrayList<Consumer<Double>> onShot = new ArrayList<Consumer<Double>>();

	public List<Consumer<Double>> getOnShotCallbacks() {
	    return onShot;
    }
    
	public void addOnShot(Consumer<Double> v) {
	    onShot.add(v);
    }
    
	public boolean removeOnShotCallback(Consumer<Double> v) {
	    return onShot.remove(v);
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

	public double targetRPM(double vel) {
		return 30.0 * PinvR.f(Math.pow(vel,2)
			* ((Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2) * Constants.SHOOTER_BALL_MASS) + Constants.SHOOTER_I)
			/ (Constants.SHOOTER_I * Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2))) / Math.PI;
	}

	@Override
	public void periodic() {
		double rpm = Math.abs((getRightVel() + getLeftVel()) * 0.5); //Average RPM
		if (rpm < (Math.min(Math.abs(ptarget), Math.abs(target)) * (1 - Constants.SIGNIFICANT_DROP_DETECTION_THSHLD))) {
			if (rpm < lowestRPM || lowestRPM < 0) {
				lowestRPM = rpm;
				lowestTime = System.currentTimeMillis();
			}
		} else if (Math.abs((rpm - Math.abs(target)) / target) <= (Constants.SIGNIFICANT_DROP_DETECTION_THSHLD * 0.5)) {
			ptarget = target;
		}
		if (lowestRPM >= 0) {
			if (System.currentTimeMillis() - lowestTime > Constants.LOWEST_EXPIRATION_TIME_MS) {
				//Throw out and use the current results - we're done dropping
				double omegaI = Math.abs(ptarget) * Math.PI / 30.0;
				double omegaF = lowestRPM * Math.PI / 30.0;
				double exitvel = Math.pow((Constants.SHOOTER_I / Constants.SHOOTER_BALL_MASS) * (Math.pow(omegaI, 2) - Math.pow(omegaF, 2)), 0.5);
				for (Consumer<Double> o : onShot) {
					o.accept(exitvel);
				}
				double xvalue = ((Math.pow(omegaI, 2) - Math.pow(omegaF, 2)) * ((Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2)
					* Constants.SHOOTER_BALL_MASS) + Constants.SHOOTER_I))
					/ (Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2) * Constants.SHOOTER_BALL_MASS);
				if (xvalue < 0 || xvalue > Math.pow(omegaI, 2)) {
					throw new IllegalStateException("Incorrect calculation of xvalue stuff. Value: "+ xvalue);
				}
				double yvalue = omegaI;
				xvalues.add(xvalue);
				yvalues.add(yvalue);
				runningRegression = true;
				//Run normalization process
				xvalues_normalized.clear();
				invR.reset();
				double maximum = 0;
				for (double d : xvalues) {
					if (d > maximum) maximum = d;
				}
				for (double d : xvalues) {
					xvalues_normalized.add(d / maximum); //Do normalization
				}
				//Change lowestRPM stuff
				lowestRPM = -1;
				lowestTime = 0;
			}
		}
		if (runningRegression) {
			for (int i = 0; i < Constants.REGRESSION_STEPS_PER_CYCLE; i++) {
				invR.gradStep(Constants.REGRESSION_ALPHA,xvalues_normalized,yvalues,Constants.REGRESSION_STEPSIZE,Constants.REGRESSION_NOISE);
			}
			regressionStepcount += Constants.REGRESSION_STEPS_PER_CYCLE;
			if (regressionStepcount >= Constants.REGRESSION_STEPS) {
				invR.setLowestInBuffer();
				runningRegression = false;
				//Copy parameter values over to PinvR
				for (int i=0;i<invR.parameters.length;i++) {
					PinvR.parameters[i]=invR.parameters[i];
				}
				//Finished regression
			}
		}
		NetworkTables.setLowestRPM(lowestRPM);
	}

	public void setLeftPID(double P, double I, double D) {
		setLeftPID(P, I, D, 0);
	}

	/**
	 * Set the PID constants for the left controller
	 *
	 * @param P  proportional gain
	 * @param I  integral gain
	 * @param D  derivative gain
	 * @param FF feed-forward gain
	 */
	public void setLeftPID(double P, double I, double D, double FF) {
		leftController.setP(P);
		leftController.setI(I);
		leftController.setD(D);
		leftController.setFF(FF);
	}

	public void setRightPID(double P, double I, double D) {
		setRightPID(P, I, D, 0);
	}

	/**
	 * Set PID Constants for the right controller
	 *
	 * @param P  proportional gain
	 * @param I  integral gain
	 * @param D  derivative gain
	 * @param FF feed-forward gain
	 */
	public void setRightPID(double P, double I, double D, double FF) {
		rightController.setP(P);
		rightController.setI(I);
		rightController.setD(D);
		rightController.setFF(FF);
	}


	/**
	 * Sets the target velocities for the two NEOs in revolutions per minute
	 *
	 * @param RPM speed of motors in revolutions per minute
	 */
	public void setSpeeds(double RPM) {
		target = RPM;
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
