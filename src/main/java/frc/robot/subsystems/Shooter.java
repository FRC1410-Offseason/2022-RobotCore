package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import frc.robot.NetworkTables;
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

	// Declare Motors
	private final CANSparkMax leftMotor = new CANSparkMax(SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);

	// Grab Encoders From Motors
	private final RelativeEncoder leftEncoder = leftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
	private final RelativeEncoder rightEncoder = rightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

	// Grab PID Controllers
	private final SparkMaxPIDController leftController = leftMotor.getPIDController();
	private final SparkMaxPIDController rightController = rightMotor.getPIDController();

	private double target = 0;
	private double pTarget = 0;
	private double lowestRPM = -1;
	private double lowestTime = 0;
	private final PolynomialRegression invR = new PolynomialRegression(REGRESSION_DEGREE, REGRESSION_BUFFERSIZE);
	private final PolynomialRegression pInvR = new PolynomialRegression(REGRESSION_DEGREE, 1);
	private boolean runningRegression = false;
	private int regressionStepCount = 0;
	private final List<Double> xValues = new DoubleArrayList();
	private final List<Double> xValuesNormalized = new DoubleArrayList();
	private final List<Double> yValues = new DoubleArrayList();
	private double normalization_divisor = 1.0f;
	private final List<DoubleConsumer> onShot = new ArrayList<>();
	private double heightTarget = SHOOTER_TARGET_HEIGHT_HIGH;

	/**
	 * Whether or not to target the low goal
	 * @param lowgoal
	 */
	public void setLowGoalTarget(boolean lowgoal) {
		heightTarget=lowgoal?SHOOTER_TARGET_HEIGHT_LOW:SHOOTER_TARGET_HEIGHT_HIGH;
	}

	private final GradientDescentOptimized alphaOptimizer = new GradientDescentOptimized(1, 5) {
		@Override
		public double error(Object... errorParams) {
			double distance = (double) errorParams[0];
			return Shooter.this.bounceError(distance, getParameters()[0]);
		}
	};

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
	 * @param vel Desired exit velocity of cargo
	 * @return RPM
	 */
	public double targetRPM(double vel) {
		return 30.0 * pInvR.polynomialFunction((Math.pow(vel, 2)
			* ((Math.pow(SHOOTER_WHEEL_RADIUS, 2) * SHOOTER_BALL_MASS) + SHOOTER_I)
			/ (SHOOTER_I * Math.pow(SHOOTER_WHEEL_RADIUS, 2))) / normalization_divisor) / Math.PI;
	}
	
	private double bounceError(double distanceFromCenter, double alpha) {
		double distance = distanceFromCenter - SHOOTER_DISTANCE_OFFSET;
		double h = heightTarget + SHOOTER_CONSTANT_HEIGHT_OFFSET + (SHOOTER_SIN_ALPHA_MULTIPLIER_HEIGHT_OFFSET * Math.sin(alpha));

		double beta = Math.atan((2 * h / distance - Math.tan(alpha)));
		if(beta > 0) {
			throw new IllegalStateException("Shooter intersection does not have a positive beta value.");
		}
		
		double v = (Math.sqrt(-SHOOTER_ACCELERATION) * distance) / (Math.cos(alpha) * Math.sqrt(2 * distance * Math.tan(alpha) - 2.0 * h));
		
		double verticalVelocity = -Math.sqrt(2.0 * SHOOTER_ACCELERATION * h
				+ Math.pow(v * Math.sin(alpha), 2.0));
		double horizontalVelocity = Math.cos(alpha)*v;
		double velocityAtTarget = Math.sqrt(Math.pow(horizontalVelocity, 2) + Math.pow(verticalVelocity, 2));

		return velocityAtTarget; //We want to minimize velocity at target.
	}
	
	/**
	 * Return an array of [vel (m/s), optimized angle (radians) from horizontal] given distance in meters.
	 */
	public double[] distanceTargeting(double distance) {
		alphaOptimizer.getParameters()[0] = Math.toRadians(80); // Yes, it's stupid. But yes, it works.
		try {
			for(int i = 0; i < SHOOTER_ALPHA_OPTIMIZER_STEPS; i++) {
				alphaOptimizer.gradStep(SHOOTER_ALPHA_OPTIMIZER_ALPHA, REGRESSION_STEPSIZE, 0, distance);
			}
			
			alphaOptimizer.setLowestInBuffer(); // No noise - so this shouldn't help or hurt anything
		} catch (IllegalStateException e) {
			// We hit a problem - our beta value is >0.
			System.err.println("Invalid beta value when running gradient steps: ");
			e.printStackTrace();
			System.err.println("Defaulting to default arm angle.");
			alphaOptimizer.getParameters()[0] = SHOOTER_MAX_ALPHA;
		}
		
		double alpha = alphaOptimizer.getParameters()[0];
		alpha = Math.max(alpha, SHOOTER_MIN_ALPHA);
		alpha = Math.min(alpha, SHOOTER_MAX_ALPHA);
		
		double h = heightTarget + SHOOTER_CONSTANT_HEIGHT_OFFSET + (SHOOTER_SIN_ALPHA_MULTIPLIER_HEIGHT_OFFSET * Math.sin(alpha));
		double v = (Math.sqrt(-SHOOTER_ACCELERATION) * distance) / (Math.cos(alpha) * Math.sqrt(2 * distance * Math.tan(alpha) - 2 * h));
		double beta = Math.atan((2 * h / distance - Math.tan(alpha)));
		
		if (Double.isNaN(v) || beta > 0) {
			// Invalid trajectory - something is broken
			throw new IllegalStateException("Unable to generate a valid trajectory.");
		}
		
		return new double[] { v, alphaOptimizer.getParameters()[0] };
	}
	@Override
	public void periodic() {
		// Get average RPM between right + left.
		double rpm = Math.abs((getRightVel() + getLeftVel()) * 0.5); // Average RPM
		// If we drop below a proportional threshold
		// Use pTarget for comparison, and it's not updated when target is updated.
		if (rpm < (Math.min(Math.abs(pTarget), Math.abs(target)) * (1 - SIGNIFICANT_DROP_DETECTION_THRESHOLD))) {
			if (rpm < lowestRPM || lowestRPM < 0) {
				lowestRPM = rpm;
				lowestTime = System.currentTimeMillis();
			}
		// When rpm gets close enough to the actual target
		} else if (Math.abs((rpm - Math.abs(target)) / target) <= (SIGNIFICANT_DROP_DETECTION_THRESHOLD * 0.5)) {
			// Update pTarget
			pTarget = target;
		}
		
		// If we have a lowestRPM
		if (lowestRPM >= 0) {
			// If it's been LOWEST_EXPIRATION_TIME_MS since we collected our last point
			if (System.currentTimeMillis() - lowestTime > LOWEST_EXPIRATION_TIME_MS) {
				// Throw out and use the current results - we're done dropping

				// Compute omega values
				double omegaI = Math.abs(pTarget) * Math.PI / 30.0;
				double omegaF = lowestRPM * Math.PI / 30.0;
				// Find exit velocity ((I/m)(w_i^2-w_f^2))^(1/2)
				double exitVelocity = Math.sqrt(SHOOTER_I / SHOOTER_BALL_MASS * (Math.pow(omegaI, 2) - Math.pow(omegaF, 2)));

				// At this point, we know that a ball has passed through the flywheels, so we increment the shot count
				shotCount++;

        		// Run callbacks
				for (DoubleConsumer callback : onShot) {
					callback.accept(exitVelocity);
				}
				// Get the abstract value w^2T(w) as an x value
				double xValue = ((Math.pow(omegaI, 2) - Math.pow(omegaF, 2)) * ((Math.pow(SHOOTER_WHEEL_RADIUS, 2)
					* SHOOTER_BALL_MASS) + SHOOTER_I))
					/ (Math.pow(SHOOTER_WHEEL_RADIUS, 2) * SHOOTER_BALL_MASS);

				if (xValue < 0 || xValue > Math.pow(omegaI, 2)) {
					throw new IllegalStateException("Incorrect calculation of xvalue stuff. Value: " + xValue);
				}
				// Add values
				xValues.add(xValue);
				yValues.add(omegaI);

				// Start out regression - reset everything
				runningRegression = true;
				xValuesNormalized.clear();
				invR.reset();
				// Run normalization process
				// Find maximum
				double maximum = 0;
				for (double value : xValues) {
					if (value > maximum) maximum = value;
				}
				// Divide everything
				for (double value : xValues) {
					xValuesNormalized.add(value / maximum); // Do normalization
				}
				normalization_divisor=maximum;
				// Sync to networktables
				// NetworkTables.setLowestRPM(lowestRPM);
				// TODO: Add network tables or figure out another way to log the lowest RPM
				// Change lowestRPM stuff to reset the state
				lowestRPM = -1;
				lowestTime = 0;
			}
		}

		if (runningRegression) {
			// Run some steps per cycle
			for (int i = 0; i < REGRESSION_STEPS_PER_CYCLE; i++) {
				invR.gradStep(
						REGRESSION_ALPHA,
						REGRESSION_STEPSIZE, REGRESSION_NOISE,
						xValuesNormalized, yValues
				);
			}

			regressionStepCount += REGRESSION_STEPS_PER_CYCLE;
			if (regressionStepCount >= REGRESSION_STEPS) {
				// Set to the lowest in buffer to undo randomness error
				invR.setLowestInBuffer();
				runningRegression = false;

				// Copy parameter values over to PinvR so we don't try and get targets while we're running stuff
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

	/**
	 * Whether the flywheels are close enough to their target velocity to start shooting
	 * @return boolean True -> We are good to shoot, False -> Can't start shooting yet
	 */
	public boolean isAtTarget() {
		return Math.abs((getLeftVel() + getRightVel()) / 2 - target) < SHOOTER_RPM_TOLERANCE;
	}

}
