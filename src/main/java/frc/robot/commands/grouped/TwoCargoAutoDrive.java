package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.LimelightAnglePID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.util.Trajectories;

public class TwoCargoAutoDrive extends ParallelCommandGroup {

    public TwoCargoAutoDrive(Trajectories trajectories, Drivetrain drivetrain, Limelight limelight) {
        drivetrain.gyro.reset();
        trajectories.generateAuto();
        trajectories.setStartingAutonomousPose(trajectories.twoBall);
        
        addCommands(
            // Drivetrain
            new SequentialCommandGroup(
                trajectories.twoBallCommand,
                trajectories.driveToShootCommand,
                new RunCommand(()-> drivetrain.tankDriveVolts(0, 0)),
                new LimelightAnglePID(limelight, drivetrain),
                new RunCommand(() -> drivetrain.tankDriveVolts(0, 0))
            )
        );
    }
}
