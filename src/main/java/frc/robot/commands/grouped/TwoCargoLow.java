package frc.robot.commands.grouped;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
						trajectories.lowHighTwoBallCommand,
						new ExtendIntake(intakeFlipper),
						new SetShooterArmAngle(shooterArm, SHOOTER_ARM_INTAKE_ANGLE),

						new SequentialCommandGroup(
								new WaitCommand(1),
								new ParallelCommandGroup(
										new SetIntakeSpeed(intake, 1, 3),
										new RunStorageForTime(storage, 3, 1)
								)
						)
				),
				new ParallelCommandGroup(
						trajectories.twoLowBackToHubCommand,
						new SetShooterArmAngle(shooterArm, SHOOTER_ARM_MAX_ANGLE),
						new RetractIntake(intakeFlipper)
				),
				new LowHubShoot(shooter, shooterArm, storage, RPM)
		);
	}
}
