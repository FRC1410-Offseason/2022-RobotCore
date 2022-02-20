package frc.robot.commands.looped;

import frc.robot.NetworkTables;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robotmap.Constants.DT200HZ;


public class DrivetrainSimulation extends CommandBase {
    private final Drivetrain drivetrain;

    public DrivetrainSimulation(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
		// We can probably move a lot of this to methods in the drivetrain itself
		// The important thing about having it in here is that it runs faster
		// We can replace all of this with a couple of function calls
        drivetrain.drivetrainSimulator.setInputs(
            drivetrain.leftLeader.get() * RobotController.getBatteryVoltage(),
            drivetrain.rightLeader.get() * RobotController.getBatteryVoltage());
        drivetrain.drivetrainSimulator.update(DT200HZ / 1000.0);

        drivetrain.yaw.set(-drivetrain.drivetrainSimulator.getHeading().getDegrees());

        drivetrain.leftEncoderPosition = drivetrain.drivetrainSimulator.getLeftPositionMeters();
        drivetrain.leftEncoderVelocity = drivetrain.drivetrainSimulator.getLeftVelocityMetersPerSecond();
        drivetrain.rightEncoderPosition = drivetrain.drivetrainSimulator.getRightPositionMeters();
        drivetrain.rightEncoderVelocity = drivetrain.drivetrainSimulator.getRightVelocityMetersPerSecond();

        drivetrain.leftEncoderSim.setDistance(drivetrain.leftEncoderPosition);
        drivetrain.leftEncoderSim.setRate(drivetrain.leftEncoderVelocity);
        drivetrain.rightEncoderSim.setDistance(drivetrain.rightEncoderPosition);
        drivetrain.rightEncoderSim.setRate(drivetrain.rightEncoderVelocity);

        drivetrain.poseEstimator.update(drivetrain.drivetrainSimulator.getHeading(), drivetrain.getWheelSpeeds(),
            drivetrain.leftEncoder.getDistance(), drivetrain.rightEncoder.getDistance());
        drivetrain.fieldSim.setRobotPose(drivetrain.getPoseEstimation());
        NetworkTables.setPoseEstimation(drivetrain.getPoseEstimation().getX(), drivetrain.getPoseEstimation().getY(),
            drivetrain.getPoseEstimation().getRotation().getDegrees(), 0, 0);
    }
}
