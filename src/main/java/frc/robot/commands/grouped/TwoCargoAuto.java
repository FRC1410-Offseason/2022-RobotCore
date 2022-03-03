package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterArm;
import frc.robot.subsystems.IntakeFlipper;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.actions.ExtendIntake;
import frc.robot.commands.actions.RunStorageForTime;
import frc.robot.commands.actions.SetIntakeSpeed;
import frc.robot.commands.actions.SetShooterArmAngle;
import frc.robot.commands.actions.SetShooterRPM;
import frc.robot.util.Trajectories;

public class TwoCargoAuto extends ParallelCommandGroup {

    public TwoCargoAuto(Trajectories trajectories, Drivetrain drivetrain, Intake intake, Storage storage, ShooterArm shooterArm, Shooter shooter, IntakeFlipper intakeFlipper, Limelight limelight, double RPM) {
        drivetrain.gyro.reset();
        trajectories.generateAuto();
        trajectories.setStartingAutonomousPose(trajectories.twoBall);
        shooterArm.resetEncoders(0);
        
        
        addCommands(
            // Drivetrain
            new SequentialCommandGroup(
                trajectories.twoBallCommand,
                new RunCommand(()-> drivetrain.tankDriveVolts(0, 0))
            ),
            // // Intake Deploy
            // new ExtendIntake(intakeFlipper),
            // // Intake
            // new SequentialCommandGroup(
            //     new WaitCommand(0.5),
            //     new SetIntakeSpeed(intake, 1, 3)
            // ),
            // // Storage
            // new SequentialCommandGroup(
            //     new WaitCommand(0.5),
            //     new RunStorageForTime(storage, 3, -1),
            //     new WaitCommand(0.5),
            //     new RunStorageForTime(storage, 0.2, 1),
            //     new WaitCommand(0.3),
            //     new RunStorageForTime(storage, 1, -1)
            // ),
            new SequentialCommandGroup(
                new WaitCommand(trajectories.twoBall.getTotalTimeSeconds() + 1),
                new LimelightShoot(drivetrain, limelight, shooter, storage, RPM),
                new RunCommand(()-> drivetrain.tankDriveVolts(0, 0))
            ),
            // Shooter Arm
            new SequentialCommandGroup(
                new SetShooterArmAngle(shooterArm, -16)
            ),
            // Other shooter arm
            new SequentialCommandGroup(
                new WaitCommand(trajectories.twoBall.getTotalTimeSeconds()),
                new SetShooterArmAngle(shooterArm, -16)
            )
        );
    }
}
