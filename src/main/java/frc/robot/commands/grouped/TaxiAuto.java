package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Trajectories;

public class TaxiAuto extends SequentialCommandGroup {

	public TaxiAuto(Trajectories trajectories, Drivetrain drivetrain) {
		drivetrain.gyro.reset();
		trajectories.generateAuto();
		trajectories.setStartingAutonomousPose(trajectories.straightline);

		addCommands(
				trajectories.straightlineCommand,
				new InstantCommand(()-> drivetrain.tankDriveVolts(0, 0))
		);
	}
}
