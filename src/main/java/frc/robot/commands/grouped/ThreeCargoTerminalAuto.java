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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterArm;
import frc.robot.subsystems.Storage;
import frc.robot.util.Trajectories;

public class ThreeCargoTerminalAuto extends ParallelCommandGroup {
    private double upperTarmacToUpperCargoShotDuration;
    private double upperCargoShotToUpperFieldDuration;
    private double upperFieldToTerminalDuration;
    private double upperTarmacToTerminalDuration;
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

    public ThreeCargoTerminalAuto(Trajectories trajectories, Intake intake, Shooter shooter, ShooterArm shooterArm, Storage storage) {        
        trajectories.generateAuto();
        trajectories.setStartingAutonomousPose(trajectories.upperTarmacToUpperCargoShot);
        trajectories.generateConfig(
            NetworkTables.getVelocityConstraint(),
            NetworkTables.getAccelerationConstraint(),
            NetworkTables.getCentripetalAccelerationConstraint());
        
        upperTarmacToUpperCargoShotDuration = trajectories.upperTarmacToUpperCargoShot.getTotalTimeSeconds();
        upperCargoShotToUpperFieldDuration = trajectories.upperCargoToUpperField.getTotalTimeSeconds();
        upperFieldToTerminalDuration = trajectories.upperFieldToTerminalShot.getTotalTimeSeconds();

        upperCargoShootDuration = intakeToStorageDuration + shooterArmLiftDuration + doubleShootDuration;

        upperTarmacToTerminalDuration = upperTarmacToUpperCargoShotDuration + upperCargoShootDuration + 
            upperCargoShotToUpperFieldDuration + upperFieldToTerminalDuration;
        
        addCommands(
            // Intake Flipper - Works in theory
            new ExtendIntake(intake),
            // Intake - Works in theory
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpperCargoShotDuration - 0.45),
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
                    new WaitCommand(upperTarmacToUpperCargoShotDuration - shooterSpinupDuration),
                    new SetShooterRPM(shooter, upperCargoShotRPM),
                    new WaitCommand(upperTarmacToUpperCargoShotDuration + upperCargoShootDuration + 0.3),
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
                    new WaitCommand(upperTarmacToUpperCargoShotDuration + intakeToStorageDuration),
                    new SetShooterArmAngle(shooterArm, highestShooterArmAngle)
                ),
                new SequentialCommandGroup(
                    new WaitCommand(upperTarmacToUpperCargoShotDuration + upperCargoShootDuration),
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
                    new WaitCommand(upperTarmacToUpperCargoShotDuration + intakeToStorageDuration + shooterArmLiftDuration),
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
                trajectories.upperFieldToTerminalShotCommand
                // Maybe the 0 tank drive volts
            )
		);
    }
}
