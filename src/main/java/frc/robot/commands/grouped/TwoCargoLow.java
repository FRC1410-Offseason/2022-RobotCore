package frc.robot.commands.grouped;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.actions.*;
import frc.robot.commands.looped.LowerShooterArmConstant;
import frc.robot.commands.looped.RaiseShooterArmConstant;
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
//				new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
                new LowHubShoot(shooter, shooterArm, storage, RPM),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                trajectories.lowHighTwoBallCommand,
                                new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0))
                        ),
						new SequentialCommandGroup(
							new ParallelRaceGroup(
									new WaitCommand(1),
									new LowerShooterArmConstant(shooterArm)
							),
							new WaitCommand(0.5),
                        	new ExtendIntake(intakeFlipper)
						),

                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new WaitCommand(trajectories.lowHighTwoBall.getTotalTimeSeconds() / 2),
                                new ParallelCommandGroup(
                                        new SetIntakeSpeed(intake, 1, 2.5),
                                        new RunStorageForTime(storage, 1.5, 1)
                                )
                        )
                ),
                new ParallelCommandGroup(
                        trajectories.twoLowBackToHubCommand,
						new RetractIntake(intakeFlipper),
						new WaitCommand(0.5),
						new ParallelRaceGroup(
								new WaitCommand(1),
								new RaiseShooterArmConstant(shooterArm)
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
