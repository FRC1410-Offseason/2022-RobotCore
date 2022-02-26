package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeFlipper;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.actions.ExtendIntake;
import frc.robot.commands.actions.RunStorageForTime;
import frc.robot.commands.actions.SetIntakeSpeed;
import frc.robot.commands.actions.SetShooterRPM;
import frc.robot.util.Trajectories;

public class TwoCargoAutoNoSA extends ParallelCommandGroup {

    public TwoCargoAutoNoSA(Trajectories trajectories, Drivetrain drivetrain, Intake intake, Storage storage, Shooter shooter, IntakeFlipper intakeFlipper, Limelight limelight) {
        drivetrain.gyro.reset();
        trajectories.generateAuto();
        trajectories.setStartingAutonomousPose(trajectories.twoBall);
        double driveDuration = trajectories.twoBall.getTotalTimeSeconds();
        
        addCommands(
            // Drivetrain
            new SequentialCommandGroup(
                trajectories.twoBallCommand,
                new RunCommand(()-> drivetrain.tankDriveVolts(0, 0))
            ),
            // Intake Deploy
            new ExtendIntake(intakeFlipper),
            // Intake
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new SetIntakeSpeed(intake, 1, 3)
            ),
            // Storage
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new RunStorageForTime(storage, 3, -1)
            ),
            // Limelight Shoot
            new SequentialCommandGroup(
                new SetShooterRPM(shooter, -1000),
                new WaitCommand(3.5),
                new LimelightShoot(drivetrain, limelight, shooter, storage)
            )
        );
    }
}
