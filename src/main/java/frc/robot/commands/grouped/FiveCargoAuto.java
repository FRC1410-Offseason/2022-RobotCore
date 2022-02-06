package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.actions.ExtendIntake;
import frc.robot.commands.actions.RunStorage;
import frc.robot.commands.actions.SetIntakeSpeed;
import frc.robot.commands.actions.SetShooterArmAngle;
import frc.robot.commands.actions.SetShooterRPM;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterArm;
import frc.robot.subsystems.Storage;
import frc.robot.util.Trajectories;

public class FiveCargoAuto extends ParallelCommandGroup {
    private double upperTarmacToUpperCargoShootDuration;
    private double upperCargoShootToUpperFieldDuration;
    private double upperFieldToUpRightCargoDuration;
    private double upRightCargoToTerminalShootDuration;

    private double upperTarmacToUpRightCargoDuration;
    private double upperTarmacToTerminalDuration;

    // ALL MADE UP NUMBERS!
    private final double upRightCargoDuration = 0.4;
    private final double upperCargoShootDuration = 1.7;
     
    private final double intakeToStorageDuration = 0.5;
    private final double shooterSpinupDuration = 2;
    private final double shooterArmLiftDuration = 1;
    private final double shooterArmLowerDuration = 0.7;
    private final double doubleShootDuration = 1;

    private final double lowestShooterArmAngle = 10;
    private final double highestIntakingShooterArmAngle = 24;
    private final double highestShooterArmAngle = 54.3;

    private final double upperCargoShotRPM = 3000;
    private final double terminalShotRPM = 4700; 

    public FiveCargoAuto(Trajectories trajectories, Intake intake, Shooter shooter, ShooterArm shooterArm, Storage storage) {        
        upperTarmacToUpperCargoShootDuration = trajectories.upperTarmacToUpperCargoShoot.getTotalTimeSeconds();
        upperCargoShootToUpperFieldDuration = trajectories.upperCargoToUpperField.getTotalTimeSeconds();
        upperFieldToUpRightCargoDuration = trajectories.upperFieldToUpRightCargo.getTotalTimeSeconds();
        upRightCargoToTerminalShootDuration = trajectories.upRightCargoToTerminalShoot.getTotalTimeSeconds();

        upperTarmacToUpRightCargoDuration = upperTarmacToUpperCargoShootDuration + upperCargoShootDuration +
            upperCargoShootToUpperFieldDuration + upperFieldToUpRightCargoDuration;
        upperTarmacToTerminalDuration = upperTarmacToUpRightCargoDuration + upRightCargoDuration + upRightCargoToTerminalShootDuration;
        
        addCommands(
            // Intake Flipper
            new ExtendIntake(intake),
            // Intake
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpperCargoShootDuration - 0.15),
                    new SetIntakeSpeed(intake, 1, 1)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpRightCargoDuration - 0.15),
                    new SetIntakeSpeed(intake, 1, 1)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration - 0.15),
                    new SetIntakeSpeed(intake, 1, 4)
                )
            ),
            // Shooter
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new SetShooterRPM(shooter, upperCargoShotRPM),
                    new WaitCommand(upperTarmacToUpperCargoShootDuration + upperCargoShootDuration),
                    new SetShooterRPM(shooter, 0)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration - shooterSpinupDuration),
                    new SetShooterRPM(shooter, terminalShotRPM) // Don't leave on after auto, but I don't think it will
                )
            ),
            // Shooter Arm
            new ParallelCommandGroup(
                new SetShooterArmAngle(shooterArm, highestIntakingShooterArmAngle),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpperCargoShootDuration + intakeToStorageDuration),
                    new SetShooterArmAngle(shooterArm, highestShooterArmAngle)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpperCargoShootDuration + upperCargoShootDuration),
                    new SetShooterArmAngle(shooterArm, lowestShooterArmAngle)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpRightCargoDuration - shooterArmLiftDuration),
                    new SetShooterArmAngle(shooterArm, highestIntakingShooterArmAngle)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration + intakeToStorageDuration),
                    new SetShooterArmAngle(shooterArm, highestShooterArmAngle)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration + intakeToStorageDuration + doubleShootDuration),
                    new SetShooterArmAngle(shooterArm, highestIntakingShooterArmAngle)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration + intakeToStorageDuration + doubleShootDuration + 
                        shooterArmLowerDuration + intakeToStorageDuration),
                    new SetShooterArmAngle(shooterArm, highestShooterArmAngle)
                )
            ),
            // Storage
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpperCargoShootDuration + intakeToStorageDuration - 0.25),
                    new RunStorage(storage) // Figure out a way to stop the storage
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpRightCargoDuration + intakeToStorageDuration - 0.25),
                    new RunStorage(storage)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration + intakeToStorageDuration - 0.25),
                    new RunStorage(storage)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToTerminalDuration + intakeToStorageDuration + doubleShootDuration + 
                        shooterArmLowerDuration + intakeToStorageDuration - 0.25),
                    new RunStorage(storage) // Find a way to stop storage after auto, if necessary
                )
            ),
            // Drivetrain
            trajectories.FiveCargo()
        );
    }
}
