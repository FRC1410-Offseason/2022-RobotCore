package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.*;
import frc.robot.subsystems.*;

import java.util.ArrayList;

public class LimelightShoot extends SequentialCommandGroup {

    public LimelightShoot(Drivetrain drivetrain, Limelight limelight, Shooter shooter, Storage storage, double RPM) {

		ArrayList<Command> toRun = new ArrayList<>();

		toRun.add(new ParallelCommandGroup(
				new ShooterSpinup(shooter, RPM),
				new LimelightAnglePID(limelight, drivetrain)
		));

		toRun.add(new Shoot(shooter, storage));

		addCommands(toRun.toArray(Command[]::new));
    }
}
