package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.actions.ExtendIntake;
import frc.robot.commands.actions.RunStorageForTime;
import frc.robot.commands.actions.SetIntakeSpeed;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeFlipper;
import frc.robot.subsystems.Limelight;

import frc.robot.util.Trajectories;

import static frc.robotmap.Constants.*;

public class TwoCargoAutoNoSA extends ParallelCommandGroup {

	public TwoCargoAutoNoSA(Trajectories trajectories, Drivetrain drivetrain, Intake intake, Storage storage, Shooter shooter, IntakeFlipper intakeFlipper, Limelight limelight, double RPM) {
		drivetrain.gyro.reset();
		trajectories.generateAuto();
		trajectories.setStartingAutonomousPose(trajectories.twoBall);

		addCommands(
				// Drivetrain
				new SequentialCommandGroup(
						trajectories.twoBallCommand,
						new InstantCommand(()-> drivetrain.tankDriveVolts(0, 0))
				),
				 // Intake Deploy
				 new ExtendIntake(intakeFlipper),
				 // Intake
				 new SequentialCommandGroup(
				     new WaitCommand(0.5),
				     new SetIntakeSpeed(intake, 1, 3)
				 ),
				 // Storage
				 new SequentialCommandGroup(
				     new WaitCommand(0.5),
				     new RunStorageForTime(storage, 3, STORAGE_INTAKE_SPEED),
				     new WaitCommand(0.5),
				     new RunStorageForTime(storage, 0.2, -STORAGE_RUN_SPEED),
				     new WaitCommand(0.3),
				     new RunStorageForTime(storage, 1, STORAGE_RUN_SPEED)
				 ),
				// Shooter
				new SequentialCommandGroup(
						new WaitCommand(trajectories.twoBall.getTotalTimeSeconds()),
						new LimelightShoot(drivetrain, limelight, shooter, storage, RPM),
						new RunCommand(()-> drivetrain.tankDriveVolts(0, 0))
				)
		);
	}
}
