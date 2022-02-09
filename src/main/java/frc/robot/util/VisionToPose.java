package frc.robot.util;

public class VisionToPose {    
    // Radius is the distance from the limelight to the vision target
    // Distance is the distance from the limelight to the center of the robot
    // Distance could be a constant because of our shooting only happening at 54deg

    public double getCameraX(double radius, double angularTheta, double cameraTheta) {
        return (-radius) * Math.cos(angularTheta + cameraTheta);
    }

    public double getCameraY(double radius, double angularTheta, double cameraTheta) {
        return (-radius) * Math.sin(angularTheta + cameraTheta);
    }

    public double getX(double cameraX, double angularTheta, double distance) {
        return cameraX - (distance * Math.cos(angularTheta));
    }
    public double getX(double radius, double angularTheta, double cameraTheta, double distance) {
        return (-radius * Math.cos(angularTheta + cameraTheta)) - (distance * Math.cos(angularTheta));
    }

    public double getY(double cameraY, double angularTheta, double distance) {
        return cameraY - (distance * Math.sin(angularTheta));
    }
    public double getY(double radius, double angularTheta, double cameraTheta, double distance) {
        return (-radius * Math.sin(angularTheta + cameraTheta)) - (distance * Math.sin(angularTheta));
    }
}
