package frc.robot.commands.grouped;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.actions.*;
import frc.robot.subsystems.*;
import frc.robot.util.Trajectories;

import static frc.robotmap.Constants.*;

public class TwoCargoLowTime extends SequentialCommandGroup {

    public TwoCargoLowTime(
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
                    new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.tankDriveVolts(4, 4)),
                        new WaitCommand(2.3),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
                        new WaitCommand(1),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(-5, -5)),
                        new WaitCommand(2.03),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
						new SequentialCommandGroup(
							new RetractIntake(intakeFlipper),
							new SetShooterArmAngle(shooterArm, SHOOTER_ARM_MAX_ANGLE)),
                        new WaitCommand(1),
                        new LowHubShoot(shooter, shooterArm, storage, RPM),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(5, 5)),
                        new WaitCommand(1.45),
                        new RunCommand(() -> drivetrain.tankDriveVolts(0, 0))
                    )
                ),
				new SequentialCommandGroup(
					new SetShooterArmAngle(shooterArm, SHOOTER_ARM_INTAKE_ANGLE),
                	new ExtendIntake(intakeFlipper)
				),

                new SequentialCommandGroup(
                   new WaitCommand(1.5),
                    new ParallelCommandGroup(
                            new SetIntakeSpeed(intake, 1, 2),
                            new RunStorageForTime(storage, 2, 1)
                    )
                )
            ),
            new WaitCommand(15)
        );
    }
}
