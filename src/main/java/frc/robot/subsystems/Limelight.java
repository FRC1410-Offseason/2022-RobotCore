package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.framework.subsystem.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import static frc.robotmap.Constants.*;

public class Limelight extends SubsystemBase {

	// The camera itself
    private final PhotonCamera limelight = new PhotonCamera("Limelight 2");

	// Stores the current values reported by the camera
    private PhotonPipelineResult latestResult;

	//TODO: This should probably be moved to a constant
    private double shooterAngle = 19;

	// The current target if there is one
    private PhotonTrackedTarget target;

    public Limelight() {
		limelight.setDriverMode(false);
	}

    @Override
    public void periodic() {        
        latestResult = limelight.getLatestResult();
        target = latestResult.getBestTarget();
    }

	public boolean hasTarget() {
		return latestResult.hasTargets();
	}

    public double getDistanceToTarget() {
        return (UPPER_HUB_HEIGHT - getLimelightHeight(shooterAngle)) / 
            Math.tan(Units.degreesToRadians(getLimelightAngle(shooterAngle) + getPitch()));
    }

	public PhotonPipelineResult getLatestResult() {
		return latestResult;
	}

	public PhotonTrackedTarget getTarget() {
		return target;
	}

	// TODO: Lots of things here that probably need to be constants
	public double getLimelightHeight(double shooterAngle) {
        return (24.08 * Math.sin(shooterAngle)) + 4.7;
    }

    public double getDistanceToRobot(double shooterAngle) {
        return (24.08 * Math.cos(shooterAngle)) - 2.527;
    }

    public double getLimelightAngle(double shooterAngle) {
        return shooterAngle - 20;
    }

    public void turnOffLEDs() {
        limelight.setLED(VisionLEDMode.kOff);
    }

    public void turnOnLEDs() {
        limelight.setLED(VisionLEDMode.kOn);
    }

    public double getPitch() {
        return target.getPitch();
    }

    public double getYaw() {
        return target.getYaw();
    }

	// TODO – docs & move to constant
    public boolean isYawAcceptable(double distance, double yaw) {
        double acceptableYaw = 5.94 - (0.161 * distance);
        if (Math.abs(yaw) < acceptableYaw) return true;
        else return false;
    }

    public double getAcceptableYaw(double distance) {
        return 5.94 - (0.161 * distance);
    }

	// TODO – move to javadoc
    // Radius is the distance from the limelight to the vision target
    // Distance is the distance from the limelight to the center of the robot
    // Distance could be a constant because our high goal shooting is only happening at 53.1deg
    // getDistanceToRobot is measuring the distance between the limelight and the center of the robot (one axis)

    public double getCameraX(double radius, double angularTheta, double cameraTheta) {
        return -radius * Math.cos(angularTheta + cameraTheta);
    }

    public double getCameraY(double radius, double angularTheta, double cameraTheta) {
        return -radius * Math.sin(angularTheta + cameraTheta);
    }

    public double getX(double cameraX, double angularTheta, double distance) {
        return cameraX - (distance * Math.cos(angularTheta));
    }

    public double getX(double radius, double angularTheta, double cameraTheta, double distance) {
        return (-radius * Math.cos(angularTheta + cameraTheta)) - (distance * Math.cos(angularTheta));
    }

    public double getX(double angularTheta, double shooterAngle) {
        return (-getDistanceToTarget() * Math.cos(angularTheta + getYaw())) - (getDistanceToRobot(shooterAngle) * Math.cos(angularTheta));
    }

    public double getY(double cameraY, double angularTheta, double distance) {
        return cameraY - (distance * Math.sin(angularTheta));
    }

    public double getY(double radius, double angularTheta, double cameraTheta, double distance) {
        return (-radius * Math.sin(angularTheta + cameraTheta)) - (distance * Math.sin(angularTheta));
    }

    public double getY(double angularTheta, double shooterAngle) {
        return (-getDistanceToTarget() * Math.sin(angularTheta + getYaw())) - (getDistanceToRobot(shooterAngle) * Math.sin(angularTheta));
    }

    public Pose2d getVisionRobotPose(double angularTheta, double shooterAngle) {
        return new Pose2d(getX(angularTheta, shooterAngle), getY(angularTheta, shooterAngle), 
            new Rotation2d(Units.degreesToRadians(angularTheta)));
        // New Pose2d and Rotation2d every hz is not great
    }
}
