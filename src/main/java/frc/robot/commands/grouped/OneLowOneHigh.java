package frc.robot.commands.grouped;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.actions.*;
import frc.robot.subsystems.*;
import frc.robot.util.Trajectories;

import static frc.robotmap.Constants.*;

public class OneLowOneHigh extends SequentialCommandGroup {

    public OneLowOneHigh(Trajectories trajectories, Drivetrain drivetrain, Intake intake, Storage storage, ShooterArm shooterArm, Shooter shooter, IntakeFlipper intakeFlipper, Limelight limelight, LEDs leds, NetworkTableEntry RPM) {
        drivetrain.gyro.reset();
        trajectories.generateAuto();
        trajectories.setStartingAutonomousPose(trajectories.twoBallGet);

        addCommands(
            new LowHubShoot(shooter, shooterArm, storage, leds, RPM),
            new ParallelCommandGroup(
                trajectories.twoBallGetCommand,
                new LowerShooterArm(shooterArm),
                new ExtendIntake(intakeFlipper),

                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new ParallelCommandGroup(new SetIntakeSpeed(intake, 1, 3), new RunStorageForTime(storage, 3, 2))
                )
            ),
            new LimelightShoot(drivetrain, intakeFlipper, limelight, shooter, shooterArm, storage, RPM)
        );
    }
}
