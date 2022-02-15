package frc.robot.subsystems;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.*;
import static frc.robotmap.Constants.*;

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
	public double leftEncoderPosition, leftEncoderVelocity, rightEncoderPosition, rightEncoderVelocity;

	public final WPI_TalonFX leftLeader = new WPI_TalonFX(DRIVETRAIN_LEFT_FRONT_MOTOR_ID);
	public final WPI_TalonFX leftFollower = new WPI_TalonFX(DRIVETRAIN_LEFT_BACK_MOTOR_ID);
	public final WPI_TalonFX rightLeader = new WPI_TalonFX(DRIVETRAIN_RIGHT_FRONT_MOTOR_ID);
	public final WPI_TalonFX rightFollower = new WPI_TalonFX(DRIVETRAIN_RIGHT_BACK_MOTOR_ID);

	private final DifferentialDrive drive;

	public final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
			new MatBuilder<>(Nat.N5(), Nat.N1()).fill(STATE_X, STATE_Y, STATE_THETA, STATE_LEFT_DIST, STATE_RIGHT_DIST),
			new MatBuilder<>(Nat.N3(), Nat.N1()).fill(LOCAL_LEFT_DIST, LOCAL_RIGHT_DIST, LOCAL_THETA),
			new MatBuilder<>(Nat.N3(), Nat.N1()).fill(VISION_X, VISION_Y, VISION_THETA), 0.005);

	public DifferentialDrivetrainSim drivetrainSimulator;

	public Field2d fieldSim;

	public final Encoder rightEncoder = new Encoder(RIGHT_ENCODER_PORTS[0], RIGHT_ENCODER_PORTS[1], false);
	public final Encoder leftEncoder = new Encoder(LEFT_ENCODER_PORTS[0], LEFT_ENCODER_PORTS[1], false);

	public EncoderSim leftEncoderSim;
	public EncoderSim rightEncoderSim;

	public final AHRS gyro = new AHRS(SPI.Port.kMXP);
	public SimDouble fusedAngle;

	public Drivetrain() {
		initializeTalonFX(leftLeader);
		initializeTalonFX(leftFollower);
		initializeTalonFX(rightLeader);
		initializeTalonFX(rightFollower);

		leftFollower.follow(leftLeader);
		rightFollower.follow(rightLeader);

		leftLeader.setInverted(true);

		drive = new DifferentialDrive(leftLeader, rightLeader);
		if (RobotBase.isSimulation()) simulationInit();

		resetEncoders();
		zeroHeading();
		
		NetworkTables.setNavXMagCalibration(gyro.isMagnetometerCalibrated());
	}

	public void simulationInit() {
		drivetrainSimulator = new DifferentialDrivetrainSim(
			DRIVETRAIN_PLANT, DCMotor.getFalcon500(2), GEARING, TRACKWIDTH, WHEEL_DIAMETER, NOISE);

		leftLeader.setInverted(false);
		leftEncoder.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);
		rightEncoder.setDistancePerPulse(ENCODER_DISTANCE_PER_PULSE);
		leftEncoderSim = new EncoderSim(leftEncoder);
		rightEncoderSim = new EncoderSim(rightEncoder);
		fusedAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "FusedHeading"));
		fieldSim = new Field2d();
		SmartDashboard.putData("Field", fieldSim);
	}

	public void periodic() {NetworkTables.setNavXMagDisturbance(gyro.isMagneticDisturbance());}

	public void initializeTalonFX(WPI_TalonFX motor) {
		motor.configFactoryDefault();
		motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
		motor.setNeutralMode(NeutralMode.Brake);
		motor.configNeutralDeadband(0.001);
	}

	public void tankDriveDeadzoned(double deadzonedLeftAxis, double deadzonedRightAxis) {
		drive.tankDrive(deadzonedLeftAxis, deadzonedRightAxis); 
		// This is squared (x^2), but we normally do sqrt(x), test both (also test both for limelight pid (but sqrt would be better there))
		drive.feed();
	}

	public void arcadeDrive(double forward, double rotation) {
		drive.arcadeDrive(forward, rotation);}

	public void tankDrive(double left, double right) {
		drive.tankDrive(left, right);
	}

	public Pose2d getPoseEstimation() {return poseEstimator.getEstimatedPosition();}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		if (RobotBase.isSimulation()) {return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());}
		else {return new DifferentialDriveWheelSpeeds(leftEncoderVelocity, rightEncoderVelocity);}
	}

	public void resetPoseEstimation(Pose2d pose) {
		if (RobotBase.isSimulation()) {
			drivetrainSimulator.setPose(pose);
			poseEstimator.resetPosition(pose, pose.getRotation());
		} else poseEstimator.resetPosition(pose, gyro.getRotation2d());
		resetEncoders();
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		leftLeader.setVoltage(leftVolts);
		rightLeader.setVoltage(rightVolts);
		drive.feed();
	}

	public void resetEncoders() {
		if (RobotBase.isSimulation()) {leftEncoder.reset(); rightEncoder.reset();}
		leftLeader.setSelectedSensorPosition(0);
		leftFollower.setSelectedSensorPosition(0);
		rightLeader.setSelectedSensorPosition(0);
		rightFollower.setSelectedSensorPosition(0);
	}

	public void setCoast() {leftLeader.setNeutralMode(NeutralMode.Coast); rightLeader.setNeutralMode(NeutralMode.Coast);}

	public void setBrake() {leftLeader.setNeutralMode(NeutralMode.Brake); rightLeader.setNeutralMode(NeutralMode.Brake);}

	public void zeroHeading() {
		gyro.reset();
	}
}
