package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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

import static frc.robotmap.Constants.*;

public class TwoCargoAuto extends ParallelCommandGroup {

    public TwoCargoAuto(Trajectories trajectories, Drivetrain drivetrain, Intake intake, Storage storage, ShooterArm shooterArm, Shooter shooter, IntakeFlipper intakeFlipper, Limelight limelight, double RPM) {
        drivetrain.gyro.reset();
        trajectories.generateAuto();
        trajectories.setStartingAutonomousPose(trajectories.twoBall);
        shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);
        
        
        addCommands(
            // Drivetrain - Trajectories
            new SequentialCommandGroup(
                new WaitCommand(1),
                trajectories.twoBallCommand,
                new InstantCommand(()-> drivetrain.tankDriveVolts(0, 0)),
                new WaitCommand(1),
                new LimelightShoot(drivetrain, limelight, shooter, shooterArm, storage, RPM),
                new RunCommand(()-> drivetrain.tankDriveVolts(0, 0))
            ),
            // Intake Deploy
            new ExtendIntake(intakeFlipper),
            // Intake
            new SequentialCommandGroup(
                new WaitCommand(1.5),
                new SetIntakeSpeed(intake, 1, 2)
            ),
            // Storage
            new SequentialCommandGroup(
                new WaitCommand(1.75),
                new RunStorageForTime(storage, 2, 1)
            ),
            // Shooter
            new SequentialCommandGroup(
                new WaitCommand(1),
                new SetShooterRPM(shooter, -1000),
                new WaitCommand(2.75),
                new SetShooterRPM(shooter, 0)
            ),
            // Shooter Arm
            new SetShooterArmAngle(shooterArm, SHOOTER_ARM_INTAKE_ANGLE)            
        );
    }
}