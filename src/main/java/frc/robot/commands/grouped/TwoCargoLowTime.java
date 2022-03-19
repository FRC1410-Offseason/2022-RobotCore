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
        trajectories.setStartingAutonomousPose(trajectories.twoBallGet);

        addCommands(
            new LowHubShoot(shooter, shooterArm, storage, RPM),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new SequentialCommandGroup(
                        new InstantCommand(() -> drivetrain.tankDriveVolts(4, 4)),
                        new WaitCommand(2.32),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
                        new WaitCommand(1),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(-5, -5)),
                        new WaitCommand(1.8),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
						new SequentialCommandGroup(
							new RetractIntake(intakeFlipper),
							new WaitCommand(0.5),
							new ParallelRaceGroup(
								new WaitCommand(1),
								new RaiseShooterArm(shooterArm)
							)
//							new SetShooterArmAngle(shooterArm,	 SHOOTER_ARM_MAX_ANGLE)),
						),
                        new WaitCommand(3),
                        new LowHubShoot(shooter, shooterArm, storage, RPM),
                        new InstantCommand(() -> drivetrain.tankDriveVolts(5, 5)),
                        new WaitCommand(1.45),
                        new RunCommand(() -> drivetrain.tankDriveVolts(0, 0))
                    )
                ),
				new SequentialCommandGroup(
					new ParallelRaceGroup(
							new WaitCommand(1),
							new LowerShooterArm(shooterArm)
					),
					new WaitCommand(0.5),
                	new ExtendIntake(intakeFlipper)
				),

                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    new ParallelCommandGroup(
                            new SetIntakeSpeed(intake, 1, 2.5),
                            new RunStorageForTime(storage, 2.5, 1)
                    )
                )
            ),
            new WaitCommand(15)
        );
    }
}
