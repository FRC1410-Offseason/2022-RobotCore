package frc.robot.commands.looped;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterArm;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robotmap.Constants.*;

public class PoseEstimation extends CommandBase {
    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private final ShooterArm shooterArm;
    private final Timer timer = new Timer();
    private double imageCaptureTime;
    private double currentPitch = 0, currentYaw = 0, lastPitch, lastYaw;

    public PoseEstimation(Drivetrain drivetrain, Limelight limelight, ShooterArm shooterArm) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.shooterArm = shooterArm;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        lastPitch = currentPitch;
        lastYaw = currentYaw;
        currentPitch = limelight.getPitch();
        currentYaw = limelight.getYaw();

        imageCaptureTime = timer.getFPGATimestamp() - limelight.getLatestResult().getLatencyMillis();
        drivetrain.leftEncoderPosition = (drivetrain.leftLeader.getSelectedSensorPosition(0) +
            drivetrain.leftFollower.getSelectedSensorPosition(0)) * ENCODER_CONSTANT / 2;
        drivetrain.leftEncoderVelocity = (drivetrain.leftLeader.getSelectedSensorVelocity(0) +
            drivetrain.leftFollower.getSelectedSensorVelocity(0)) * ENCODER_CONSTANT / 2 * 10;
        drivetrain.rightEncoderPosition = (drivetrain.rightLeader.getSelectedSensorPosition(0) +
            drivetrain.rightFollower.getSelectedSensorPosition(0)) * ENCODER_CONSTANT / 2;
        drivetrain.rightEncoderVelocity = (drivetrain.rightLeader.getSelectedSensorVelocity(0) +
            drivetrain.rightFollower.getSelectedSensorVelocity(0)) * ENCODER_CONSTANT / 2 * 10;

        drivetrain.poseEstimator.update(drivetrain.gyro.getRotation2d(), drivetrain.getWheelSpeeds(),
            drivetrain.leftEncoderPosition, drivetrain.rightEncoderPosition);
        if (lastPitch != currentPitch && lastYaw != currentYaw && limelight.getTarget() != null) {
            drivetrain.poseEstimator.addVisionMeasurement(limelight.getVisionRobotPose(drivetrain.gyro.getRotation2d().getDegrees(),
                Units.radiansToDegrees(shooterArm.getEncoderPosition())), imageCaptureTime);
        }
    }
}
