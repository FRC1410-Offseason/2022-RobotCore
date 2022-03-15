package frc.robot.commands.grouped;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.util.Trajectories;

import java.util.ArrayList;

import static frc.robotmap.Constants.SHOOTER_ARM_MAX_ANGLE;

public class ThreeCargoLow extends SequentialCommandGroup {

	public ThreeCargoLow(
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
				new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)),
				new LowHubShoot(shooter, shooterArm, storage, RPM)
		);
	}
}
