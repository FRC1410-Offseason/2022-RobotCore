package frc.robot.commands.looped;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robotmap.Constants.*;

public class PoseEstimation extends CommandBase {
    private final Drivetrain drivetrain;

    public PoseEstimation(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.leftEncoderPosition = (drivetrain.leftLeader.getSelectedSensorPosition(0) +
            drivetrain.leftFollower.getSelectedSensorPosition(0)) * ENCODER_CONSTANT / 2;
        drivetrain.leftEncoderVelocity = (drivetrain.leftLeader.getSelectedSensorVelocity(0) +
            drivetrain.leftFollower.getSelectedSensorVelocity(0)) * ENCODER_CONSTANT / 2 * 10;
        drivetrain.rightEncoderPosition = (drivetrain.rightLeader.getSelectedSensorPosition(0) +
            drivetrain.rightFollower.getSelectedSensorPosition(0)) * ENCODER_CONSTANT / 2;
        drivetrain.rightEncoderVelocity = (drivetrain.rightLeader.getSelectedSensorVelocity(0) +
            drivetrain.rightFollower.getSelectedSensorVelocity(0)) * ENCODER_CONSTANT / 2 * 10;

        drivetrain.poseEstimator.update(drivetrain.m_gyro.getRotation2d(), drivetrain.getWheelSpeeds(),
            drivetrain.leftEncoderPosition, drivetrain.rightEncoderPosition);
    }
}
