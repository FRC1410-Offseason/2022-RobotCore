package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robotmap.Constants.*;
import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.*;

public class Drivetrain extends SubsystemBase {

	private final WPI_TalonFX mLeftLeader = new WPI_TalonFX(DRIVETRAIN_LEFT_FRONT_MOTOR_ID);
	private final WPI_TalonFX mLeftFollower = new WPI_TalonFX(DRIVETRAIN_LEFT_BACK_MOTOR_ID);

	private final WPI_TalonFX mRightLeader = new WPI_TalonFX(DRIVETRAIN_RIGHT_FRONT_MOTOR_ID);
	private final WPI_TalonFX mRightFollower = new WPI_TalonFX(DRIVETRAIN_RIGHT_BACK_MOTOR_ID);

	private final AHRS mGyro = new AHRS(SerialPort.Port.kMXP);
	private final SimDouble mGyroSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));

	private final DifferentialDrive mDrive = new DifferentialDrive(mLeftLeader, mRightLeader);

	private final DifferentialDrivePoseEstimator mPoseEstimator = new DifferentialDrivePoseEstimator(new Rotation2d(), new Pose2d(),
			new MatBuilder<>(Nat.N5(), Nat.N1()).fill(STATE_X, STATE_Y, STATE_THETA, STATE_LEFT_DIST, STATE_RIGHT_DIST),
			new MatBuilder<>(Nat.N3(), Nat.N1()).fill(LOCAL_LEFT_DIST, LOCAL_RIGHT_DIST, LOCAL_THETA),
			new MatBuilder<>(Nat.N3(), Nat.N1()).fill(VISION_X, VISION_Y, VISION_THETA));
	private final Field2d mFieldSim;
	private DifferentialDrivetrainSim mSim;
	private TalonFXSimCollection mLeftEncoderSim;
	private TalonFXSimCollection mRightEncoderSim;

	public Drivetrain() {
		mLeftLeader.configFactoryDefault();
		mLeftFollower.configFactoryDefault();
		mLeftLeader.setNeutralMode(NeutralMode.Brake);
		mLeftFollower.setNeutralMode(NeutralMode.Brake);
		mLeftFollower.follow(mLeftLeader);
		mLeftLeader.setInverted(true);
		mLeftFollower.setInverted(InvertType.FollowMaster);

		mRightLeader.configFactoryDefault();
		mRightFollower.configFactoryDefault();
		mRightLeader.setNeutralMode(NeutralMode.Brake);
		mRightFollower.setNeutralMode(NeutralMode.Brake);
		mRightFollower.follow(mRightLeader);
		mRightFollower.setInverted(true);

		if (RobotBase.isSimulation()) {
			simInit();
		}

		mFieldSim = new Field2d();
		SmartDashboard.putData("Field", mFieldSim);


	}

	private void simInit() {
		mSim = new DifferentialDrivetrainSim(
				DRIVETRAIN_PLANT,
				DCMotor.getFalcon500(2),
				GEARING,
				TRACKWIDTH,
				WHEEL_DIAMETER,
				NOISE
		);

		mLeftLeader.setInverted(false);

		mLeftEncoderSim = mLeftLeader.getSimCollection();
		mRightEncoderSim = mRightLeader.getSimCollection();
	}

	@Override
	public void periodic() {
		mPoseEstimator.update(
				mGyro.getRotation2d(),
				getWheelSpeeds(),
				(mLeftLeader.getSelectedSensorPosition() / 2048 / GEARING) * Math.PI * WHEEL_DIAMETER,
				(mRightLeader.getSelectedSensorPosition() / 2048 / GEARING) * Math.PI * WHEEL_DIAMETER
		);
	}

	@Override
	public void simulationPeriodic() {
		mSim.setInputs(
				mLeftLeader.get() * RobotController.getBatteryVoltage(),
				mRightLeader.get() * RobotController.getBatteryVoltage()
		);
		mSim.update(0.02);

		mGyroSim.set(-mSim.getHeading().getDegrees());

		mFieldSim.setRobotPose(mPoseEstimator.getEstimatedPosition());

		mLeftEncoderSim.setIntegratedSensorRawPosition((int) (mSim.getLeftPositionMeters() / ENCODER_CONSTANT));
		mRightEncoderSim.setIntegratedSensorRawPosition((int) (mSim.getRightPositionMeters() / ENCODER_CONSTANT));

		mLeftEncoderSim.setIntegratedSensorVelocity((int) (mSim.getLeftVelocityMetersPerSecond() / 10 / ENCODER_CONSTANT));
		mRightEncoderSim.setIntegratedSensorVelocity((int) (mSim.getRightVelocityMetersPerSecond() / 10 / ENCODER_CONSTANT));
	}

	public Pose2d getPose() {
		return mPoseEstimator.getEstimatedPosition();
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		System.out.println("Left: " + ((((mLeftLeader.getSelectedSensorVelocity() / 2048) * 10) / GEARING) * Math.PI * WHEEL_DIAMETER) + " Right: " + ((((mRightLeader.getSelectedSensorVelocity() / 2048) * 10) / GEARING) * Math.PI * WHEEL_DIAMETER));
		return new DifferentialDriveWheelSpeeds(
				(((mLeftLeader.getSelectedSensorVelocity() / 2048) * 10) / GEARING) * Math.PI * WHEEL_DIAMETER,
				(((mRightLeader.getSelectedSensorVelocity() / 2048) * 10) / GEARING) * Math.PI * WHEEL_DIAMETER
		);
	}

	public void resetPoseEstimator(Pose2d pose) {
		if (RobotBase.isSimulation()) {
			mSim.setPose(pose);
			mPoseEstimator.resetPosition(pose, pose.getRotation());
		} else {
			mPoseEstimator.resetPosition(pose, mGyro.getRotation2d());
		}
		resetEncoders();
	}

	public void arcadeDrive(double forward, double rotation) {
		mDrive.arcadeDrive(forward, rotation);
	}

	public void tankDrive(double left, double right) {
		mDrive.tankDrive(left, right);
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		mLeftLeader.setVoltage(leftVolts);
		mRightLeader.setVoltage(rightVolts);
		mDrive.feed();
	}

	public void resetEncoders() {
		mLeftLeader.setSelectedSensorPosition(0);
		mRightLeader.setSelectedSensorPosition(0);
	}

	public void setCoast() {
	}

	public void setBrake() {
	}

	public void zeroHeading() {
		mGyro.reset();
	}
}

