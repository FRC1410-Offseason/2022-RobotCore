package frc.robot.subsystems;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.framework.subsystem.SubsystemBase;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {

	/**
	 * These are used for odometry / pose estimation
	 * This is not best practice, but it works, and we don't have time to fix it atm
	 */
	public double leftEncoderPosition, leftEncoderVelocity, rightEncoderPosition, rightEncoderVelocity;

	/**
	 * Motors
	 */
	public final WPI_TalonFX leftLeader = new WPI_TalonFX(DRIVETRAIN_LEFT_FRONT_MOTOR_ID);
	public final WPI_TalonFX leftFollower = new WPI_TalonFX(DRIVETRAIN_LEFT_BACK_MOTOR_ID);
	public final WPI_TalonFX rightLeader = new WPI_TalonFX(DRIVETRAIN_RIGHT_FRONT_MOTOR_ID);
	public final WPI_TalonFX rightFollower = new WPI_TalonFX(DRIVETRAIN_RIGHT_BACK_MOTOR_ID);

	/**
	 * Wrapper class for motors that makes them easier to use
	 */
	private final DifferentialDrive drive;

    private boolean isInverted = false;

	/**
	 * Odometry but GUD, uses some fancy vector math to accurately update the estimated position of the robot on the field
	 */
	public final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
			new MatBuilder<>(Nat.N5(), Nat.N1()).fill(STATE_X, STATE_Y, STATE_THETA, STATE_LEFT_DIST, STATE_RIGHT_DIST),
			new MatBuilder<>(Nat.N3(), Nat.N1()).fill(LOCAL_LEFT_DIST, LOCAL_RIGHT_DIST, LOCAL_THETA),
			new MatBuilder<>(Nat.N3(), Nat.N1()).fill(VISION_X, VISION_Y, VISION_THETA), 10.0 / 1000.0);

	/**
	 * Gyro and sim handle for the gyro
	 */
	public final AHRS gyro = new AHRS(SPI.Port.kMXP);

	public Drivetrain() {
		//Config motors
		initializeTalonFX(leftLeader);
		initializeTalonFX(leftFollower);
		initializeTalonFX(rightLeader);
		initializeTalonFX(rightFollower);

		leftLeader.setSafetyEnabled(false);
		leftFollower.setSafetyEnabled(false);

		rightLeader.setSafetyEnabled(false);
		rightFollower.setSafetyEnabled(false);

		leftFollower.follow(leftLeader);
		rightFollower.follow(rightLeader);

		leftLeader.setInverted(true);
		leftFollower.setInverted(InvertType.FollowMaster);

		drive = new DifferentialDrive(leftLeader, rightLeader);

		drive.setSafetyEnabled(false);

		//Reset everything to make sure it's good to go
		resetEncoders();
		zeroHeading();
	}

	@Override
	public void periodic() {
        System.out.println("Front Left: " + leftLeader.getSelectedSensorPosition(0));
        System.out.println("Back Left: " + leftFollower.getSelectedSensorPosition(0));
		drive.feedWatchdog();
	}

	/**
	 * Sets up a falcon:
	 * - Performs a factory reset
	 * - Makes sure it's reading from the correct encoder
	 * - Sets the neutral mode to break
	 * - And configures a neutral deadband
	 * @param motor a motor object representing a TalonFX / Falcon 500
	 */
	public void initializeTalonFX(WPI_TalonFX motor) {
		motor.configFactoryDefault();
		motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		motor.configNeutralDeadband(0.001);
	}

	/**
	 * Run the drivetrain with a tank drive scheme
	 * @param left -1 to 1 representing left velocity
	 * @param right -1 to 1 representing right velocity
	 */
	public void tankDrive(double left, double right, boolean squared) {
		if (isInverted) {
            drive.tankDrive(-right, -left, squared);
        } else {
            drive.tankDrive(left, right, squared);
        }
		drive.feed();
	}

    public void flip() {
        isInverted = !isInverted;
    }

	/**
	 * Get the estimated position of the robot
	 * @return a Pose2d object representing the position of the robot
	 */
	public Pose2d getPoseEstimation() {
		return poseEstimator.getEstimatedPosition();
	}

	/**
	 * Get the current speeds of the wheels
	 * @return a DifferentialDriveWheelSpeeds object that contains the wheels speeds in m/s
	 */
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(leftEncoderVelocity, rightEncoderVelocity);
	}

	/**
	 * Reset the pose of the robot on the field
	 * @param pose a starting pose
	 */
	public void resetPoseEstimation(Pose2d pose) {
		poseEstimator.resetPosition(pose, gyro.getRotation2d());
		resetEncoders();
	}

	/**
	 * Drive the robot in tank mode using voltage inputs
	 * @param leftVolts -12 to 12
	 * @param rightVolts -12 to 12
	 */
	public void tankDriveVolts(double leftVolts, double rightVolts) {
        System.out.println("Applied LEFT voltage: " + leftVolts);
        System.out.println("Applied RIGHT voltage: " + rightVolts);

		leftLeader.setVoltage(leftVolts);
		rightLeader.setVoltage(rightVolts);
		drive.feed();
	}

	/**
	 * Reset the encoders
	 */
	public void resetEncoders() {
		leftLeader.setSelectedSensorPosition(0);
		rightLeader.setSelectedSensorPosition(0);
	}

	/**
	 * Set the motors to coast mode
	 */
	public void setCoast() {
		leftLeader.setNeutralMode(NeutralMode.Coast);
		rightLeader.setNeutralMode(NeutralMode.Coast);
		leftFollower.setNeutralMode(NeutralMode.Coast);
		rightFollower.setNeutralMode(NeutralMode.Coast);
	}

	/**
	 * Set the motors to brake mode
	 */
	public void setBrake() {
		leftLeader.setNeutralMode(NeutralMode.Brake);
		rightLeader.setNeutralMode(NeutralMode.Brake);
		leftFollower.setNeutralMode(NeutralMode.Brake);
		rightFollower.setNeutralMode(NeutralMode.Brake);
	}

	/**
	 * Reset the stored heading of the robot
	 */
	public void zeroHeading() {
		gyro.reset();
	}
}
