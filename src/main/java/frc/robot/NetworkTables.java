package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import static frc.robotmap.Tuning.*;

public class NetworkTables {

	static final NetworkTableInstance instance = NetworkTableInstance.getDefault();
	static final NetworkTable table = instance.getTable("Dashboard Data");
	static final NetworkTable robotState = instance.getTable("Robot State");
	static final NetworkTable limelight = instance.getTable("photonvision/Limelight 2");
	static final NetworkTableEntry autoList, autoChooser; // Auto
	static final NetworkTableEntry x, y, theta, wheelLeft, wheelRight, navxMagDisturbance, navxMagCalibrated,
		leftEncoderDistanceAverage, rightEncoderDistanceAverage; // Drivetrain
	static final NetworkTableEntry pitch, yaw, visionDistance, limelightAngleKP, limelightAngleKI, limelightAngleKD; // Limelight
	static final NetworkTableEntry shooterTargetRPM, leftShooterRPM, leftShooterP, leftShooterI, leftShooterD, leftShooterFF,
		rightShooterP, rightShooterI, rightShooterD, rightShooterFF, rightShooterRPM, lowestRPM; // Shooter
	static final NetworkTableEntry correctColor, colorReading, lineBroken, storageSpeed; // Storage
	static final NetworkTableEntry shooterArmAngle, shooterArmLocked; // Shooter Arm
	static final NetworkTableEntry pressure; // Pneumatics
	static final NetworkTableEntry intakeDeployed, intakeSpeed; // Intake
	static final NetworkTableEntry leftWinchHeight, leftWinchLocked, rightWinchHeight, rightWinchLocked; // Winches
	static final NetworkTableEntry leftTelescopingArmHeight, leftTelescopingArmLocked,
		rightTelescopingArmHeight, rightTelescopingArmLocked; // Telescoping Arms
	static final NetworkTableEntry poseEstimationUpdateRate; // Hz Calcs

	static {
		// Autonomous
		autoList = table.getEntry("Auto List");
		autoChooser = table.getEntry("Auto Chooser");
		// Drivetrain
		x = robotState.getEntry("X");
		y = robotState.getEntry("Y");
		theta = robotState.getEntry("Heading");
		wheelLeft = robotState.getEntry("Wheel Speed Left");
		wheelRight = robotState.getEntry("Wheel Speed Right");
		navxMagDisturbance = robotState.getEntry("Navx Magnetometer Is Disturbed");
		navxMagCalibrated = robotState.getEntry("Navx Magnetometer is Calibrated");
		leftEncoderDistanceAverage = robotState.getEntry("Left Encoder Distance Average");
		rightEncoderDistanceAverage = robotState.getEntry("Right Encoder Distance Average");
		// Limelight
		visionDistance = robotState.getEntry("Vision Distance");
		limelightAngleKP = robotState.getEntry("Limelight Angle P Gain");
		limelightAngleKI = robotState.getEntry("Limelight Angle I Gain");
		limelightAngleKD = robotState.getEntry("Limelight Angle D Gain");
		pitch = limelight.getEntry("targetPitch");
		yaw = limelight.getEntry("targetYaw");
		// Shooter
		shooterTargetRPM = robotState.getEntry("Target RPM");
		leftShooterRPM = robotState.getEntry("Left RPM");
		leftShooterP = robotState.getEntry("Left Shooter P");
		leftShooterI = robotState.getEntry("Left Shooter I");
		leftShooterD = robotState.getEntry("Left Shooter D");
		leftShooterFF = robotState.getEntry("Left Shooter FF");
		rightShooterRPM = robotState.getEntry("Right RPM");
		rightShooterP = robotState.getEntry("Right Shooter P");
		rightShooterI = robotState.getEntry("Right Shooter I");
		rightShooterD = robotState.getEntry("Right Shooter D");
		rightShooterFF = robotState.getEntry("Right Shooter FF");
		lowestRPM = robotState.getEntry("Lowest Shooter RPM Drop");
		// Storage
		correctColor = robotState.getEntry("Correct Color");
		colorReading = robotState.getEntry("Detected Color");
		lineBroken = robotState.getEntry("Line is Broken");
		storageSpeed = robotState.getEntry("Storage RPM");
		// Shooter Arm
		shooterArmAngle = robotState.getEntry("Shooter Arm Angle");
		shooterArmLocked = robotState.getEntry("Shooter Arm is Locked");
		// Pneumatics
		pressure = robotState.getEntry("Pressure");
		// Intake
		intakeDeployed = robotState.getEntry("Intake is Deployed");
		intakeSpeed = robotState.getEntry("Intake Speed");
		// Winches
		leftWinchHeight = robotState.getEntry("Left Winch Height");
		rightWinchHeight = robotState.getEntry("Right Winch Height");
		leftWinchLocked = robotState.getEntry("Left Winch is Locked");
		rightWinchLocked = robotState.getEntry("Right Winch is Locked");
		// Telescoping Arms
		leftTelescopingArmHeight = robotState.getEntry("Left Telescoping Arm Height");
		rightTelescopingArmHeight = robotState.getEntry("Right Telescoping Arm Height");
		leftTelescopingArmLocked = robotState.getEntry("Left Telescoping Arm is Locked");
		rightTelescopingArmLocked = robotState.getEntry("Right Telescoping Arm is Locked");
		// Hz Calcs
		poseEstimationUpdateRate = robotState.getEntry("Pose Estimation Updates In Last Second");

		// Initializing
		// Autonomous
		autoChooser.setDouble(0);
		// Drivetrain
		x.setDouble(0);
		y.setDouble(0);
		theta.setDouble(0);
		wheelLeft.setDouble(0);
		wheelRight.setDouble(0);
		navxMagDisturbance.setBoolean(false);
		navxMagCalibrated.setBoolean(false);
		leftEncoderDistanceAverage.setDouble(0);
		rightEncoderDistanceAverage.setDouble(0);
		// Limelight
		visionDistance.setDouble(0);
		limelightAngleKP.setDouble(LIMELIGHT_ANGLE_KP);
		limelightAngleKI.setDouble(LIMELIGHT_ANGLE_KI);
		limelightAngleKD.setDouble(LIMELIGHT_ANGLE_KD);
		// Shooter
		shooterTargetRPM.setDouble(0);
		leftShooterRPM.setDouble(0);
		leftShooterP.setDouble(0);
		leftShooterI.setDouble(0);
		leftShooterD.setDouble(0);
		leftShooterFF.setDouble(0);
		rightShooterRPM.setDouble(0);
		rightShooterP.setDouble(0);
		rightShooterI.setDouble(0);
		rightShooterD.setDouble(0);
		rightShooterFF.setDouble(0);
		lowestRPM.setDouble(0);
		// Storage
		correctColor.setString("");
		colorReading.setString("");
		lineBroken.setBoolean(false);
		storageSpeed.setDouble(0);
		// Shooter Arm
		shooterArmAngle.setDouble(0);
		shooterArmLocked.setBoolean(false);
		// Pneumatics
		pressure.setDouble(0);
		// Intake
		intakeDeployed.setBoolean(false);
		intakeSpeed.setDouble(0);
		// Winches
		leftWinchHeight.setDouble(0);
		rightWinchHeight.setDouble(0);
		leftWinchLocked.setBoolean(false);
		rightWinchLocked.setBoolean(false);
		// Telescoping Arms
		leftTelescopingArmHeight.setDouble(0);
		rightTelescopingArmHeight.setDouble(0);
		leftTelescopingArmLocked.setBoolean(false);
		rightTelescopingArmLocked.setBoolean(false);
		// Hz Calcs
		poseEstimationUpdateRate.setDouble(0);
	}

	// Autonomous
	public static void setAutoList(String[] AUTO_LIST) {autoList.setStringArray(AUTO_LIST);}

	public static double getAutoChooser() {return autoChooser.getDouble(0);}

	// Drivetrain
	public static void setPoseEstimation(double X, double Y, double THETA, double wheelSpeedLeft, double wheelSpeedRight) {
		x.setDouble(X);
		y.setDouble(Y);
		theta.setDouble(THETA);
		wheelLeft.setDouble(wheelSpeedLeft);
		wheelRight.setDouble(wheelSpeedRight);
	}
	public static void setNavXMagDisturbance(boolean isDisturbed) {navxMagDisturbance.setBoolean(isDisturbed);}
	public static void setNavXMagCalibration(boolean isCalibrated) {navxMagCalibrated.setBoolean(isCalibrated);}

	public static void setEncoderDistance(double leftEncoderDistance, double rightEncoderDistance) {
		leftEncoderDistanceAverage.setDouble(leftEncoderDistance);
		rightEncoderDistanceAverage.setDouble(rightEncoderDistance);
	}
	// Limelight
	public static void setVisionDistance(double distance) {visionDistance.setDouble(distance);}

	public static double getLimelightAngleKP() {
		return limelightAngleKP.getDouble(LIMELIGHT_ANGLE_KP);
	}
	public static double getLimelightAngleKI() {
		return limelightAngleKP.getDouble(LIMELIGHT_ANGLE_KI);
	}
	public static double getLimelightAngleKD() {
		return limelightAngleKP.getDouble(LIMELIGHT_ANGLE_KD);
	}

	public static double getPitch() {
		return pitch.getDouble(0);
	}

	public static double getYaw() {
		return yaw.getDouble(0);
	}

	// Shooter
	public static double getShooterTargetRPM() {
		return shooterTargetRPM.getDouble(0);
	}
	public static void setShooterTargetRPM(double RPM) {
		shooterTargetRPM.setDouble(RPM);
	}

	public static double getLeftShooterP() {
		return leftShooterP.getDouble(0);
	}
	public static double getLeftShooterI() {
		return leftShooterI.getDouble(0);
	}
	public static double getLeftShooterD() {
		return leftShooterD.getDouble(0);
	}
	public static double getLeftShooterFF() {
		return leftShooterFF.getDouble(0);
	}
	public static double getLeftShooterRPM() {
		return leftShooterRPM.getDouble(0);
	}

	public static double getRightShooterP() {
		return leftShooterP.getDouble(0);
	}
	public static double getRightShooterI() {
		return leftShooterI.getDouble(0);
	}
	public static double getRightShooterD() {
		return leftShooterD.getDouble(0);
	}
	public static double getRightShooterFF() {
		return leftShooterFF.getDouble(0);
	}
	public static double getRightShooterRPM() {
		return leftShooterRPM.getDouble(0);
	}

	public static void setLeftShooterPIDFF(double P, double I, double D, double FF) {
		leftShooterP.setDouble(P);
		leftShooterI.setDouble(I);
		leftShooterD.setDouble(D);
		leftShooterFF.setDouble(FF);
	}

	public static void setRightShooterPIDFF(double P, double I, double D, double FF) {
		rightShooterP.setDouble(P);
		rightShooterI.setDouble(I);
		rightShooterD.setDouble(D);
		rightShooterFF.setDouble(FF);
	}

	public static void setLowestRPM(double RPM) {lowestRPM.setDouble(RPM);}

	// Storage
	public static String getCorrectColor() {
		return correctColor.getString("");
	}
	public static void setCorrectColor(String color) {
		correctColor.setString(color);
	}

	public static String getColorReading() {
		return colorReading.getString("");
	}
	public static void setColorReading(String color) {
		colorReading.setString(color);
	}

	public static boolean getLineBroken() {
		return lineBroken.getBoolean(false);
	}
	public static void setLineBroken(boolean broken) {
		lineBroken.setBoolean(broken);
	}

	public static void setStorageSpeed(double RPM) {
		storageSpeed.setDouble(RPM);
	}

	// Shooter Arm
	public static double getShooterAngle() {
		return shooterArmAngle.getDouble(0);
	}
	public static void setShooterAngle(double angle) {
		shooterArmAngle.setDouble(angle);
	}

	public static void setShooterArmLocked(boolean locked) {
		shooterArmLocked.setBoolean(locked);
	}
	public static boolean getShooterLocked() {
		return shooterArmLocked.getBoolean(false);
	}

	// Pneumatics
	public static double getPressure(AnalogInput pressureInput) {
		return (50 * pressureInput.getVoltage()) - 25;
	}
	public static void setPressure(AnalogInput pressureInput) {
		pressure.setDouble((50 * pressureInput.getVoltage()) - 25);
	}

	// Intake
	public static double getIntakeRPM() {
		return intakeSpeed.getDouble(0);
	}
	public static void setIntakeSpeed(double speed) {
		intakeSpeed.setDouble(speed);
	}

	public static boolean getIntakeDeployed() {
		return intakeDeployed.getBoolean(false);
	}
	public static void setIntakeDeployed(boolean deployed) {
		intakeDeployed.setBoolean(deployed);
	}


	// Winches
	public static double getLeftWinchHeight() {
		return leftWinchHeight.getDouble(0);
	}
	public static void setLeftWinchHeight(double height) {
		leftWinchHeight.setDouble(height);
	}

	public static boolean getLeftWinchLocked() {
		return rightWinchLocked.getBoolean(false);
	}
	public static void setLeftWinchLocked(boolean locked) {
		leftWinchLocked.setBoolean(locked);
	}

	public static double getRightWinchHeight() {
		return rightWinchHeight.getDouble(0);
	}
	public static void setRightWinchHeight(double height) {
		rightWinchHeight.setDouble(height);
	}

	public static boolean getRightWinchLocked() {
		return rightWinchLocked.getBoolean(false);
	}
	public static void setRightWinchLocked(boolean locked) {
		rightWinchLocked.setBoolean(locked);
	}

	// Telescoping Arms
	public static double getLeftTelescopingArmHeight() {
		return leftTelescopingArmHeight.getDouble(0);
	}
	public static void setLeftTelescopingArmHeight(double height) {
		leftTelescopingArmHeight.setDouble(height);
	}

	public static boolean getLeftTelescopingArmLocked() {
		return rightTelescopingArmLocked.getBoolean(false);
	}
	public static void setLeftTelescopingArmLocked(boolean locked) {
		leftTelescopingArmLocked.setBoolean(locked);
	}

	public static double getRightTelescopingArmHeight() {
		return rightTelescopingArmHeight.getDouble(0);
	}
	public static void setRightTelescopingArmHeight(double height) {
		rightTelescopingArmHeight.setDouble(height);
	}

	public static boolean getRightTelescopingArmLocked() {
		return rightTelescopingArmLocked.getBoolean(false);
	}
	public static void setRightTelescopingArmLocked(boolean locked) {
		rightTelescopingArmLocked.setBoolean(locked);
	}
	// Hz Calcs
	public static void setPoseEstimationHz(double hz) {
		poseEstimationUpdateRate.setDouble(hz);
	}
}
