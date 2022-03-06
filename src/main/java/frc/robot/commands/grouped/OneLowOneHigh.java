package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.actions.*;
import frc.robot.commands.looped.RunIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterArm;
import frc.robot.subsystems.IntakeFlipper;
import frc.robot.subsystems.Limelight;
import frc.robot.util.Trajectories;

import static frc.robotmap.Constants.*;

public class OneLowOneHigh extends SequentialCommandGroup {

    public OneLowOneHigh(Trajectories trajectories, Drivetrain drivetrain, Intake intake, Storage storage, ShooterArm shooterArm, Shooter shooter, IntakeFlipper intakeFlipper, Limelight limelight, double RPM) {
        drivetrain.gyro.reset();
        trajectories.generateAuto();
        trajectories.setStartingAutonomousPose(trajectories.lowHighTwoBall);
        shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);
        
        
        addCommands(
            
            new LowHubShoot(shooter, shooterArm, storage, RPM),

            new ParallelCommandGroup(
                trajectories.lowHighTwoBallCommand,

                new SetShooterArmAngle(shooterArm, SHOOTER_ARM_INTAKE_ANGLE),
                new ExtendIntake(intakeFlipper),

                new SequentialCommandGroup(
                    new WaitCommand(1),

                    new ParallelCommandGroup(new SetIntakeSpeed(intake, 1, 3), new RunStorageForTime(storage, 3, 2))
                )
            ),

            new LimelightShoot(drivetrain, intakeFlipper, limelight, shooter, shooterArm, storage, RPM)

        );
    }
}
