package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.NetworkTables;
import frc.robot.commands.actions.ExtendIntake;
import frc.robot.commands.actions.RunStorageForTime;
import frc.robot.commands.actions.SetIntakeSpeed;
import frc.robot.commands.actions.SetShooterArmAngle;
import frc.robot.commands.actions.SetShooterRPM;
import frc.robot.subsystems.*;
import frc.robot.util.Trajectories;

public class FiveCargoAutoCornerStart extends ParallelCommandGroup {
    private double upRightTarmacToUpperCargoShootDuration;
    private double upperCargoShootToUpperFieldDuration;
    private double upperFieldToUpRightCargoDuration;
    private double upRightCargoToTerminalShootDuration;

    private double upperTarmacToUpRightCargoDuration;
    private double upperTarmacToTerminalDuration;
    private double upperTarmacToFifthCargoReady;

	private double upRightCargoDuration;
	private double upperCargoShootDuration;

    // ALL MADE UP NUMBERS!     
    private final double intakeToStorageDuration = 0.5;
    private final double shooterSpinupDuration = 2;
    private final double shooterArmLiftDuration = 0.55; // From sim
    private final double shooterArmLowerDuration = 0.515; // From sim
    private final double doubleShootDuration = 1;

    private final double highestIntakingShooterArmAngle = 24;
    private final double highestShooterArmAngle = 53.1;// From CAD

    private final double upperCargoShotRPM = 3000;
    private final double terminalShotRPM = 4700; 

    public FiveCargoAutoCornerStart(Trajectories trajectories, Intake intake, IntakeFlipper intakeFlipper, Shooter shooter, ShooterArm shooterArm, Storage storage) {
        trajectories.generateAuto();
        trajectories.setStartingAutonomousPose(trajectories.upRightTarmacToUpperCargoShot);
        trajectories.generateConfig(
            NetworkTables.getVelocityConstraint(),
            NetworkTables.getAccelerationConstraint(),
            NetworkTables.getCentripetalAccelerationConstraint());
        
        upRightCargoToTerminalShootDuration = trajectories.upRightTarmacToUpperCargoShot.getTotalTimeSeconds();
        upperCargoShootToUpperFieldDuration = trajectories.upperCargoToUpperField.getTotalTimeSeconds();
        upperFieldToUpRightCargoDuration = trajectories.upperFieldToUpRightCargo.getTotalTimeSeconds();
        upRightCargoToTerminalShootDuration = trajectories.upRightCargoToTerminalShot.getTotalTimeSeconds();

        upRightCargoDuration = intakeToStorageDuration - 0.25;
        upperCargoShootDuration = intakeToStorageDuration + shooterArmLiftDuration + doubleShootDuration;

        upperTarmacToUpRightCargoDuration = upRightTarmacToUpperCargoShootDuration + upperCargoShootDuration +
            upperCargoShootToUpperFieldDuration + upperFieldToUpRightCargoDuration;
        upperTarmacToTerminalDuration = upperTarmacToUpRightCargoDuration + upRightCargoDuration + upRightCargoToTerminalShootDuration;
        upperTarmacToFifthCargoReady = upperTarmacToTerminalDuration + intakeToStorageDuration + shooterArmLiftDuration + 
        doubleShootDuration + shooterArmLowerDuration;
        
        addCommands(
            // Intake Flipper - Works in theory
            new ExtendIntake(intakeFlipper),
            // Intake - Works in theory
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(upRightTarmacToUpperCargoShootDuration - 0.45),
                    new SetIntakeSpeed(intake, 1, 1)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpRightCargoDuration - 0.45),
                    new SetIntakeSpeed(intake, 1, 1)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration - 0.45),
                    new SetIntakeSpeed(intake, 1, 5)
                )
            ),
            // Shooter - Works in theory
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(upRightTarmacToUpperCargoShootDuration - shooterSpinupDuration),
                    new SetShooterRPM(shooter, upperCargoShotRPM),
                    new WaitCommand(upRightTarmacToUpperCargoShootDuration + upperCargoShootDuration + 0.3),
                    new SetShooterRPM(shooter, 0)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration + intakeToStorageDuration + shooterArmLiftDuration - shooterSpinupDuration),
                    new SetShooterRPM(shooter, terminalShotRPM),
                    new WaitCommand(doubleShootDuration + 0.3),
                    new SetShooterRPM(shooter, 0)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToFifthCargoReady + intakeToStorageDuration + shooterArmLiftDuration - shooterSpinupDuration),
                    new SetShooterRPM(shooter, terminalShotRPM),
                    new WaitCommand(doubleShootDuration + 0.3),
                    new SetShooterRPM(shooter, 0)
                )
            ),
            // Shooter Arm - Works in theory
            new ParallelCommandGroup(
                new SetShooterArmAngle(shooterArm, highestIntakingShooterArmAngle),
                new SequentialCommandGroup(
                    new WaitCommand(upRightTarmacToUpperCargoShootDuration + intakeToStorageDuration),
                    new SetShooterArmAngle(shooterArm, highestShooterArmAngle)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upRightTarmacToUpperCargoShootDuration + upperCargoShootDuration),
                    new SetShooterArmAngle(shooterArm, highestIntakingShooterArmAngle)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration + intakeToStorageDuration),
                    new SetShooterArmAngle(shooterArm, highestShooterArmAngle)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration + intakeToStorageDuration + shooterArmLiftDuration + doubleShootDuration),
                    new SetShooterArmAngle(shooterArm, highestIntakingShooterArmAngle)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToFifthCargoReady + intakeToStorageDuration),
                    new SetShooterArmAngle(shooterArm, highestShooterArmAngle)
                )
            ),
            // Storage - Works in theory
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(upRightTarmacToUpperCargoShootDuration + intakeToStorageDuration + shooterArmLiftDuration),
                    new RunStorageForTime(storage, doubleShootDuration)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration + intakeToStorageDuration + shooterArmLiftDuration),
                    new RunStorageForTime(storage, doubleShootDuration)
				),
				new SequentialCommandGroup(
					new WaitCommand(upperTarmacToFifthCargoReady + intakeToStorageDuration + shooterArmLiftDuration),
					new RunStorageForTime(storage, doubleShootDuration)
				)
            ),
            // Drivetrain - Works in theory
            new SequentialCommandGroup(
                trajectories.upRightTarmacToUpperCargoShotCommand,
                new WaitCommand(upperCargoShootDuration),
                trajectories.upperCargoToUpperFieldCommand,
                trajectories.upperFieldToUpRightCargoCommand,
                new WaitCommand(upRightCargoDuration),
                trajectories.upRightCargoToTerminalShotCommand
                // Maybe the 0 tank drive volts
            )
		);
    }
}
