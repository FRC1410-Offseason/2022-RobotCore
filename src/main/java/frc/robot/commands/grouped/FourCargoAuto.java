package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.actions.ExtendIntake;
import frc.robot.commands.actions.RunStorageForTime;
import frc.robot.commands.actions.SetIntakeSpeed;
import frc.robot.commands.actions.SetShooterArmAngle;
import frc.robot.commands.actions.SetShooterRPM;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterArm;
import frc.robot.subsystems.Storage;
import frc.robot.util.Trajectories;

public class FourCargoAuto extends ParallelCommandGroup {
    private double upperTarmacToUpperCargoShootDuration;
    private double upperCargoShootToUpperFieldDuration;
    private double upperFieldToUpRightCargoDuration;
    private double upRightCargoToTerminalShootDuration;

    private double upperTarmacToUpRightCargoDuration;
    private double upperTarmacToTerminalDuration;

	private double upRightCargoDuration;
	private double upperCargoShootDuration;

    // ALL MADE UP NUMBERS!     
    private final double intakeToStorageDuration = 0.5;
    private final double shooterSpinupDuration = 2;
    private final double shooterArmLiftDuration = 0.55; // From sim
    private final double doubleShootDuration = 1;

    private final double highestIntakingShooterArmAngle = 24;
    private final double highestShooterArmAngle = 54.3;

    private final double upperCargoShotRPM = 3000;
    private final double terminalShotRPM = 4700; 

    public FourCargoAuto(Trajectories trajectories, Intake intake, Shooter shooter, ShooterArm shooterArm, Storage storage) {        
        trajectories.generateAuto();
        trajectories.setStartingAutonomousPose();
        
        upperTarmacToUpperCargoShootDuration = trajectories.upperTarmacToUpperCargoShot.getTotalTimeSeconds();
        upperCargoShootToUpperFieldDuration = trajectories.upperCargoToUpperField.getTotalTimeSeconds();
        upperFieldToUpRightCargoDuration = trajectories.upperFieldToUpRightCargo.getTotalTimeSeconds();
        upRightCargoToTerminalShootDuration = trajectories.upRightCargoToTerminalShot.getTotalTimeSeconds();

        upRightCargoDuration = intakeToStorageDuration - 0.25;
        upperCargoShootDuration = intakeToStorageDuration + shooterArmLiftDuration + doubleShootDuration;

        upperTarmacToUpRightCargoDuration = upperTarmacToUpperCargoShootDuration + upperCargoShootDuration +
            upperCargoShootToUpperFieldDuration + upperFieldToUpRightCargoDuration;
        upperTarmacToTerminalDuration = upperTarmacToUpRightCargoDuration + upRightCargoDuration + upRightCargoToTerminalShootDuration;
        
        addCommands(
            // Intake Flipper - Works in theory
            new ExtendIntake(intake),
            // Intake - Works in theory
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpperCargoShootDuration - 0.45),
                    new SetIntakeSpeed(intake, 1, 1)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpRightCargoDuration - 0.45),
                    new SetIntakeSpeed(intake, 1, 1)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration - 0.45),
                    new SetIntakeSpeed(intake, 1, 1)
                )
            ),
            // Shooter - Works in theory
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpperCargoShootDuration - shooterSpinupDuration),
                    new SetShooterRPM(shooter, upperCargoShotRPM),
                    new WaitCommand(upperTarmacToUpperCargoShootDuration + upperCargoShootDuration + 0.3),
                    new SetShooterRPM(shooter, 0)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration + intakeToStorageDuration + shooterArmLiftDuration - shooterSpinupDuration),
                    new SetShooterRPM(shooter, terminalShotRPM),
                    new WaitCommand(doubleShootDuration + 0.3),
                    new SetShooterRPM(shooter, 0)
                )
            ),
            // Shooter Arm - Works in theory
            new ParallelCommandGroup(
                new SetShooterArmAngle(shooterArm, highestIntakingShooterArmAngle),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpperCargoShootDuration + intakeToStorageDuration),
                    new SetShooterArmAngle(shooterArm, highestShooterArmAngle)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpperCargoShootDuration + upperCargoShootDuration),
                    new SetShooterArmAngle(shooterArm, highestIntakingShooterArmAngle)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration + intakeToStorageDuration),
                    new SetShooterArmAngle(shooterArm, highestShooterArmAngle)
                )
            ),
            // Storage - Works in theory
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpperCargoShootDuration + intakeToStorageDuration + shooterArmLiftDuration),
                    new RunStorageForTime(storage, doubleShootDuration)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration + intakeToStorageDuration + shooterArmLiftDuration),
                    new RunStorageForTime(storage, doubleShootDuration)
				)
            ),
            // Drivetrain - Works in theory
            new SequentialCommandGroup(
                trajectories.upperTarmacToUpperCargoShotCommand,
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
