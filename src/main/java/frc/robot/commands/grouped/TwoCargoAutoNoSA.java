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
import frc.robot.subsystems.IntakeFlipper;
import frc.robot.subsystems.Limelight;

import frc.robot.util.Trajectories;

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
				// // Intake Deploy
				// new ExtendIntake(intakeFlipper),
				// // Intake
				// new SequentialCommandGroup(
				//     new WaitCommand(0.5),
				//     new SetIntakeSpeed(intake, 1, 3)
				// ),
				// // Storage
				// new SequentialCommandGroup(
				//     new WaitCommand(0.5),
				//     new RunStorageForTime(storage, 3, -1),
				//     new WaitCommand(0.5),
				//     new RunStorageForTime(storage, 0.2, 1),
				//     new WaitCommand(0.3),
				//     new RunStorageForTime(storage, 1, -1)
				// ),
				// Shooter
				new SequentialCommandGroup(
						// new SetShooterRPM(shooter, -1000),
						// new WaitCommand(3.5),
						// new SetShooterRPM(shooter, 0),
						// new WaitCommand(1),
						new WaitCommand(trajectories.twoBall.getTotalTimeSeconds()),
						new LimelightShoot(drivetrain, limelight, shooter, storage, RPM),
						new RunCommand(()-> drivetrain.tankDriveVolts(0, 0))
				)
		);
	}
}
