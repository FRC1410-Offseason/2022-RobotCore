package frc.robot.commands.grouped;


import edu.wpi.first.networktables.NetworkTableEntry;
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
            NetworkTableEntry RPM)
    {
        drivetrain.gyro.reset();
        trajectories.generateAuto();
        trajectories.setStartingAutonomousPose(trajectories.twoBallGet);

        addCommands(
				new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
                new LowHubShoot(shooter, shooterArm, storage, RPM),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                trajectories.twoBallGetCommand,
                                new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0))
                        ),
						new SequentialCommandGroup(
							new ParallelRaceGroup(
									new WaitCommand(SHOOTER_ARM_DOWN_TIME),
									new LowerShooterArm(shooterArm)
							),
							new WaitCommand(0.5),
                        	new ExtendIntake(intakeFlipper)
						),

                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new WaitCommand(trajectories.twoBallGet.getTotalTimeSeconds() / 2),
                                new ParallelCommandGroup(
                                        new SetIntakeSpeed(intake, 1, 2.5),
                                        new RunStorageForTime(storage, 1.5, 1)
                                )
                        )
                ),
                new ParallelCommandGroup(
                        trajectories.twoBallReturnCommand,
						new RetractIntake(intakeFlipper),
						new SequentialCommandGroup(
								new WaitCommand(1),
								new ParallelRaceGroup(
										new WaitCommand(SHOOTER_ARM_UP_TIME),
										new RaiseShooterArm(shooterArm)
								)
						)
                ),
				new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
                new LowHubShoot(shooter, shooterArm, storage, RPM),
				new InstantCommand(() -> drivetrain.tankDriveVolts(5, 5)),
				new WaitCommand(1.45),
				new RunCommand(() -> drivetrain.tankDriveVolts(0, 0)),
                new WaitCommand(15)
        );
    }
}
