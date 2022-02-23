package frc.robot.subsystems;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.*;
import static frc.robotmap.Constants.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NetworkTables;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.hal.SimDouble;
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
	// TODO: Add encoder conversion factor from calibration
	public final WPI_TalonFX leftLeader = new WPI_TalonFX(DRIVETRAIN_LEFT_FRONT_MOTOR_ID);
	public final WPI_TalonFX leftFollower = new WPI_TalonFX(DRIVETRAIN_LEFT_BACK_MOTOR_ID);
	public final WPI_TalonFX rightLeader = new WPI_TalonFX(DRIVETRAIN_RIGHT_FRONT_MOTOR_ID);
	public final WPI_TalonFX rightFollower = new WPI_TalonFX(DRIVETRAIN_RIGHT_BACK_MOTOR_ID);

	/**
	 * Wrapper class for motors that makes them easier to use
	 */
	private final DifferentialDrive drive;

	/**
	 * Odometry but GUD, uses some fancy vector math to accurately update the estimated position of the robot on the field
	 */
	public final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
			new MatBuilder<>(Nat.N5(), Nat.N1()).fill(STATE_X, STATE_Y, STATE_THETA, STATE_LEFT_DIST, STATE_RIGHT_DIST),
			new MatBuilder<>(Nat.N3(), Nat.N1()).fill(LOCAL_LEFT_DIST, LOCAL_RIGHT_DIST, LOCAL_THETA),
			new MatBuilder<>(Nat.N3(), Nat.N1()).fill(VISION_X, VISION_Y, VISION_THETA), DT200HZ / 1000);

	/**
	 * Used for simulating the drivetrain (duh)
	 */
	public DifferentialDrivetrainSim drivetrainSimulator;

	/**
	 * Used for sim, we send it to networktables to display the robot pose on a field
	 */
	public Field2d fieldSim;

	/**
	 * Only used for simulation, again not best practice, but it works
	 */
	public final Encoder rightEncoder = new Encoder(RIGHT_ENCODER_PORTS[0], RIGHT_ENCODER_PORTS[1], false);
	public final Encoder leftEncoder = new Encoder(LEFT_ENCODER_PORTS[0], LEFT_ENCODER_PORTS[1], false);

	/**
	 * Simulation stuff, do not do this, it's very bad practice
	 */
	public EncoderSim leftEncoderSim;
	public EncoderSim rightEncoderSim;

	/**
	 * Gyro and sim handle for the gyro
	 */
	public final AHRS gyro = new AHRS(SPI.Port.kMXP);
	public SimDouble yaw;

	public Drivetrain() {
		//Config motors
		initializeTalonFX(leftLeader);
		initializeTalonFX(leftFollower);
		initializeTalonFX(rightLeader);
		initializeTalonFX(rightFollower);

		leftFollower.follow(leftLeader);
		rightFollower.follow(rightLeader);

		leftLeader.setInverted(true);
		leftFollower.setInverted(InvertType.FollowMaster);

		drive = new DifferentialDrive(leftLeader, rightLeader);

		//If we are in a simulation, we have to set a couple of things up
		if (RobotBase.isSimulation()) {
			simulationInit();
		}

		//Reset everything to make sure it's good to go
		resetEncoders();
		zeroHeading();

		//Send som DATA to networktables
		NetworkTables.setNavXMagCalibration(gyro.isMagnetometerCalibrated());
	}

	/**
	 * Sets up the simulation environment for this subsystem
	 * Creates the actual simulation instance
	 * Sets up the encoders for use later
	 */
	public void simulationInit() {
		drivetrainSimulator = new DifferentialDrivetrainSim(
			DRIVETRAIN_PLANT, DCMotor.getFalcon500(2), GEARING, TRACKWIDTH, WHEEL_DIAMETER, NOISE);

		leftLeader.setInverted(false);
		leftEncoder.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);
		rightEncoder.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);
		leftEncoderSim = new EncoderSim(leftEncoder);
		rightEncoderSim = new EncoderSim(rightEncoder);
		yaw = new SimDouble(SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));
		fieldSim = new Field2d();
		SmartDashboard.putData("Field", fieldSim);
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
		motor.setNeutralMode(NeutralMode.Brake);
		motor.configNeutralDeadband(0.001);
	}

	/**
	 * Run the drivetrain in arcade mode
	 * @param forward -1 to 1 representing the desired velocity
	 * @param rotation -1 to 1 representing the desired angular velocity
	 */
	public void arcadeDrive(double forward, double rotation) {
		drive.arcadeDrive(forward, rotation);
	}

	/**
	 * Not really any different from the `tankDriveDeadzoned()` method
	 * @param left -1 to 1 representing left velocity
	 * @param right -1 to 1 representing right velocity
	 */
	public void tankDrive(double left, double right, boolean squareInputs) {
		drive.tankDrive(left, right, false);
		drive.feed();
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
		if (RobotBase.isSimulation()) {return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());}
		else {return new DifferentialDriveWheelSpeeds(leftEncoderVelocity, rightEncoderVelocity);}
	}

	/**
	 * Reset the pose of the robot on the field
	 * @param pose a starting pose
	 */
	public void resetPoseEstimation(Pose2d pose) {
		if (RobotBase.isSimulation()) {
			drivetrainSimulator.setPose(pose);
			poseEstimator.resetPosition(pose, pose.getRotation());
		} else poseEstimator.resetPosition(pose, gyro.getRotation2d());
		resetEncoders();
	}

	/**
	 * Drive the robot in tank mode using voltage inputs
	 * @param leftVolts -12 to 12
	 * @param rightVolts -12 to 12
	 */
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		leftLeader.setVoltage(leftVolts);
		rightLeader.setVoltage(rightVolts);
		drive.feed();
	}

	/**
	 * Reset the encoders
	 */
	public void resetEncoders() {
		if (RobotBase.isSimulation()) {
			leftEncoder.reset(); rightEncoder.reset();
		}
		leftLeader.setSelectedSensorPosition(0);
		leftFollower.setSelectedSensorPosition(0);
		rightLeader.setSelectedSensorPosition(0);
		rightFollower.setSelectedSensorPosition(0);
	}

	/**
	 * Set the motors to coast mode
	 */
	public void setCoast() {
		leftLeader.setNeutralMode(NeutralMode.Coast); rightLeader.setNeutralMode(NeutralMode.Coast);
	}

	/**
	 * Set the motors to brake mode
	 */
	public void setBrake() {
		leftLeader.setNeutralMode(NeutralMode.Brake); rightLeader.setNeutralMode(NeutralMode.Brake);
	}

	/**
	 * Reset the stored heading of the robot
	 */
	public void zeroHeading() {
		gyro.reset();
	}
}
