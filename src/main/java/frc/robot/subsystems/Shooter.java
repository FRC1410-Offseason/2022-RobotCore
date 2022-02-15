package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.NetworkTables;
import frc.robot.framework.subsystem.SubsystemBase;
import frc.robot.util.GradientDescentOptimized;
import frc.robot.util.PolynomialRegression;
import frc.robotmap.Constants;
import it.unimi.dsi.fastutil.doubles.DoubleArrayList;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

import static frc.robotmap.Constants.SHOOTER_RPM_TOLERANCE;
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

	private double shotCount = 0;
	private boolean outtakeQueued = false;

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

	public double getShotCount() {
		return shotCount;
	}

	public void resetShotCount() {
		shotCount = 0;
	}

	public void queueOuttake() {
		outtakeQueued = true;
	}

	public boolean isOuttakeQueued() {
		return outtakeQueued;
	}
  /**
	 * Use the regression to find a target RPM based on a target velocity in m/s.
	 * @param vel
	 * @return RPM
	 */
	public double targetRPM(double vel) {
		return 30.0 * pInvR.function(Math.pow(vel,2)
			* ((Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2) * Constants.SHOOTER_BALL_MASS) + Constants.SHOOTER_I)
			/ (Constants.SHOOTER_I * Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2))) / Math.PI;
	}
	private GradientDescentOptimized alphaOptimizer=new GradientDescentOptimized(1, 5) {
		@Override public double error(Object... errorparams) {
			Double distance = (Double)errorparams[0];
			return Shooter.this.bounceError(distance,getParameters()[0]);
		}
	};
	private double bounceError(double distance, double alpha) {
		double h = Constants.SHOOTER_TARGET_HEIGHT+Constants.SHOOTER_CONSTANT_HEIGHT_OFFSET+(Constants.SHOOTER_SIN_ALPHA_MULTIPLIER_HEIGHT_OFFSET * Math.sin(alpha));

		double beta = Math.atan((2*h/distance - Math.tan(alpha)));
		if(beta > 0) {
			throw new IllegalStateException("Shooter intersection does not have a positive beta value.");
		}
		double reflectedBeta = (2 * Constants.SHOOTER_CONE_ANGLE_RADIANS) - beta;
		double v = (Math.sqrt(-Constants.SHOOTER_ACCELERATION) * distance) / (Math.cos(alpha) * Math.sqrt(2 * distance * Math.tan(alpha) - 2.0 * h));
		double vertvel = -Math.sqrt(2.0 * Constants.SHOOTER_ACCELERATION * h
				+ Math.pow(v * Math.sin(alpha), 2.0));
		double horizvel = Math.cos(alpha)*v;
		double velAtTarget = Math.sqrt(Math.pow(horizvel,2)+Math.pow(vertvel,2));
		double twiceReflectedBeta = (-2 * Constants.SHOOTER_CONE_ANGLE_RADIANS) - reflectedBeta; //Not actually correct, but it's off by a factor of 2pi. So it ends up working.

		return velAtTarget * Math.sin(twiceReflectedBeta); //Vertical component of the projected bounce
	}
	/**
	 * Return an array of [vel (m/s), optimized angle (radians) from horizontal] given distance in meters.
	 */
	public double[] distanceTargeting(double distance) {
		alphaOptimizer.getParameters()[0] = Math.toRadians(80); //Yes, it's stupid. But yes, it works.
		try {
			for(int i=0;i<Constants.SHOOTER_ALPHA_OPTIMIZER_STEPS;i++) {
				alphaOptimizer.gradStep(Constants.SHOOTER_ALPHA_OPTIMIZER_ALPHA, Constants.REGRESSION_STEPSIZE, 0, (Double)distance);
			}
			alphaOptimizer.setLowestInBuffer(); //No noise - so this shouldn't help or hurt anything
		} catch (IllegalStateException e) {
			//We hit a problem - our beta value is >0.
			System.err.println("Invalid beta value when running gradient steps: ");
			e.printStackTrace();
			System.err.println("Defaulting to default arm angle.");
			alphaOptimizer.getParameters()[0] = Constants.SHOOTER_MAX_ALPHA;
		}
		double alpha = alphaOptimizer.getParameters()[0];
		alpha = Math.max(alpha, Constants.SHOOTER_MIN_ALPHA);
		alpha = Math.min(alpha, Constants.SHOOTER_MAX_ALPHA);
		double h = Constants.SHOOTER_TARGET_HEIGHT + Constants.SHOOTER_CONSTANT_HEIGHT_OFFSET + (Constants.SHOOTER_SIN_ALPHA_MULTIPLIER_HEIGHT_OFFSET * Math.sin(alpha));
		double v = (Math.sqrt(-Constants.SHOOTER_ACCELERATION) * distance) / (Math.cos(alpha)*Math.sqrt(2 * distance * Math.tan(alpha) - 2.0 * h));
		double beta = Math.atan((2 * h / distance - Math.tan(alpha)));
		if(v == Double.NaN || beta > 0) {
			//Invalid trajectory - something is broken
			throw new IllegalStateException("Unable to generate a valid trajectory.");
		}
		return new double[] {v, alphaOptimizer.getParameters()[0]};
	}
	@Override
	public void periodic() {
		//Get average RPM between right + left.
		double rpm = Math.abs((getRightVel() + getLeftVel()) * 0.5); //Average RPM
		//If we drop below a proportional threshold
		//Use pTarget for comparison, and it's not updated when target is updated.
		if (rpm < (Math.min(Math.abs(pTarget), Math.abs(target)) * (1 - Constants.SIGNIFICANT_DROP_DETECTION_THSHLD))) {
			if (rpm < lowestRPM || lowestRPM < 0) {
				lowestRPM = rpm;
				lowestTime = System.currentTimeMillis();
			}
		//When rpm gets close enough to the actual target
		} else if (Math.abs((rpm - Math.abs(target)) / target) <= (Constants.SIGNIFICANT_DROP_DETECTION_THSHLD * 0.5)) {
			//Update pTarget
			pTarget = target;
		}
		//If we have a lowestRPM
		if (lowestRPM >= 0) {
			//If it's been LOWEST_EXPIRATION_TIME_MS since we collected our last point
			if (System.currentTimeMillis() - lowestTime > Constants.LOWEST_EXPIRATION_TIME_MS) {
				//Throw out and use the current results - we're done dropping

				//Compute omega values
				double omegaI = Math.abs(pTarget) * Math.PI / 30.0;
				double omegaF = lowestRPM * Math.PI / 30.0;
				//Find exit velocity ((I/m)(w_i^2-w_f^2))^(1/2)
				double exitVelocity = Math.sqrt(Constants.SHOOTER_I / Constants.SHOOTER_BALL_MASS * (Math.pow(omegaI, 2) - Math.pow(omegaF, 2)));

				//At this point, we know that a ball has passed through the flywheels, so we increment the shot count
				shotCount++;

        //Run callbacks
				for (DoubleConsumer callback : onShot) {
					callback.accept(exitVelocity);
				}
				//Get the abstract value w^2T(w) as an x value
				double xValue = ((Math.pow(omegaI, 2) - Math.pow(omegaF, 2)) * ((Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2)
					* Constants.SHOOTER_BALL_MASS) + Constants.SHOOTER_I))
					/ (Math.pow(Constants.SHOOTER_WHEEL_RADIUS, 2) * Constants.SHOOTER_BALL_MASS);

				if (xValue < 0 || xValue > Math.pow(omegaI, 2)) {
					throw new IllegalStateException("Incorrect calculation of xvalue stuff. Value: " + xValue);
				}
				//Add values
				xValues.add(xValue);
				yValues.add(omegaI);

				//Start out regression - reset everything
				runningRegression = true;
				xValuesNormalized.clear();
				invR.reset();
				//Run normalization process
				//Find maximum
				double maximum = 0;
				for (double value : xValues) {
					if (value > maximum) maximum = value;
				}
				//Divide everything
				for (double value : xValues) {
					xValuesNormalized.add(value / maximum); //Do normalization
				}
				//Sync to networktables
				NetworkTables.setLowestRPM(lowestRPM);
				//Change lowestRPM stuff to reset the state
				lowestRPM = -1;
				lowestTime = 0;
			}
		}

		if (runningRegression) {
			//Run some steps per cycle
			for (int i = 0; i < Constants.REGRESSION_STEPS_PER_CYCLE; i++) {
				invR.gradStep(
						Constants.REGRESSION_ALPHA,
						Constants.REGRESSION_STEPSIZE, Constants.REGRESSION_NOISE,
						xValuesNormalized, yValues
				);
			}

			regressionStepCount += Constants.REGRESSION_STEPS_PER_CYCLE;
			if (regressionStepCount >= Constants.REGRESSION_STEPS) {
				//Set to the lowest in buffer to undo randomness error
				invR.setLowestInBuffer();
				runningRegression = false;

				//Copy parameter values over to PinvR so we don't try and get targets while we're running stuff
				System.arraycopy(invR.getParameters(), 0, pInvR.getParameters(), 0, invR.getParameters().length);
			}
		}
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

	public boolean isAtTarget() {
		return Math.abs((getLeftVel() + getRightVel()) / 2 - target) < SHOOTER_RPM_TOLERANCE;
	}

}
