package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.LimelightAnglePID;
import frc.robot.commands.actions.RunStorageForTime;
import frc.robot.commands.actions.SetShooterRPM;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

public class LimelightShoot extends ParallelCommandGroup {
    private double RPM;

    public LimelightShoot(Drivetrain drivetrain, Limelight limelight, Shooter shooter, Storage storage) {
        RPM = shooter.targetRPM(1);// Make real value

        addCommands(
            new SetShooterRPM(shooter, RPM),
            new SequentialCommandGroup(
                new LimelightAnglePID(limelight, drivetrain), 
                new RunStorageForTime(storage, 1)// tune durations
            )
        );
    }
}
