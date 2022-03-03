package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterArm;
import frc.robot.subsystems.IntakeFlipper;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.actions.ExtendIntake;
import frc.robot.commands.actions.RunStorageForTime;
import frc.robot.commands.actions.SetIntakeSpeed;
import frc.robot.commands.actions.SetShooterArmAngle;
import frc.robot.util.Trajectories;

import static frc.robotmap.Constants.*;

public class TwoCargoAuto extends ParallelCommandGroup {

	public TwoCargoAuto(Trajectories trajectories, Drivetrain drivetrain, Intake intake, Storage storage, ShooterArm shooterArm, Shooter shooter, IntakeFlipper intakeFlipper, Limelight limelight, double RPM) {
		drivetrain.gyro.reset();
		trajectories.generateAuto();
		trajectories.setStartingAutonomousPose(trajectories.twoBall);
		shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);

		addCommands(
				// Drivetrain - Trajectories
				new SequentialCommandGroup(
						trajectories.twoBallCommand,
						new RunCommand(()-> drivetrain.tankDriveVolts(0, 0))
				),
				// Limelight Shoot
				new SequentialCommandGroup(
						new WaitCommand(trajectories.twoBall.getTotalTimeSeconds() + 1),
						new LimelightShoot(drivetrain, limelight, shooter, storage, RPM),
						new RunCommand(()-> drivetrain.tankDriveVolts(0, 0))
				),
				// Intake Deploy
				new ExtendIntake(intakeFlipper),
				// Intake
				new SequentialCommandGroup(
						new WaitCommand(0.5),
						new SetIntakeSpeed(intake, INTAKE_FORWARD_SPEED, 3)
				),
				// Storage
				new SequentialCommandGroup(
						new RunStorageForTime(storage, 0.2, -STORAGE_RUN_SPEED),
						new WaitCommand(0.3),
						new RunStorageForTime(storage, 1, STORAGE_RUN_SPEED)
				),
				// Shooter Arm
				new SequentialCommandGroup(
						new SetShooterArmAngle(shooterArm, SHOOTER_ARM_INTAKE_ANGLE)
				),
				// Second Shooter Arm
				new SequentialCommandGroup(
						new WaitCommand(trajectories.twoBall.getTotalTimeSeconds()),
						new SetShooterArmAngle(shooterArm, SHOOTER_ARM_MAX_ANGLE)
				)
		);
	}
}
