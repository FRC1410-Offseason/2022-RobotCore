package frc.robot.commands.grouped;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.actions.*;
import frc.robot.subsystems.*;
import frc.robot.util.Trajectories;

import static frc.robotmap.Constants.*;

public class TwoCargoLow extends SequentialCommandGroup {

    public TwoCargoLow(
            Trajectories trajectories,
            Drivetrain drivetrain,
            Intake intake,
            Storage storage,
            ShooterArm shooterArm,
            Shooter shooter,
            IntakeFlipper intakeFlipper,
            double RPM)
    {
        drivetrain.gyro.reset();
        trajectories.generateAuto();
        trajectories.setStartingAutonomousPose(trajectories.lowHighTwoBall);
        shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);

        addCommands(
                new LowHubShoot(shooter, shooterArm, storage, RPM),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                trajectories.lowHighTwoBallCommand,
                                new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0))
                        ),
						new SequentialCommandGroup(
							new SetShooterArmAngle(shooterArm, SHOOTER_ARM_INTAKE_ANGLE),
                        	new ExtendIntake(intakeFlipper)
						),

                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new WaitCommand(trajectories.lowHighTwoBall.getTotalTimeSeconds() / 2),
                                new ParallelCommandGroup(
                                        new SetIntakeSpeed(intake, 1, 2),
                                        new RunStorageForTime(storage, 2, 1)
                                )
                        )
                ),
                new ParallelCommandGroup(
                        trajectories.twoLowBackToHubCommand,
                        new SetShooterArmAngle(shooterArm, SHOOTER_ARM_MAX_ANGLE),
                        new RetractIntake(intakeFlipper)
                ),
                new LowHubShoot(shooter, shooterArm, storage, RPM),
                new WaitCommand(15)
        );
    }
}
