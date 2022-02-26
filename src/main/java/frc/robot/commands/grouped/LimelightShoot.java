package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.*;
import frc.robot.subsystems.*;

import java.util.ArrayList;

public class LimelightShoot extends SequentialCommandGroup {

    public LimelightShoot(Drivetrain drivetrain, Limelight limelight, Shooter shooter, Storage storage) {
		// TODO: Add calculation for desired exit velocity of cargo instead of just 1
        int RPM = (int) shooter.targetRPM(1);

		ArrayList<Command> toRun = new ArrayList<>();

		toRun.add(new LimelightAnglePID(limelight, drivetrain));

		if (shooter.isOuttakeQueued()) {
			toRun.add(new Shoot(shooter, storage, RPM));
			toRun.add(new ShootOuttake(shooter, storage));
		} else {
			toRun.add(new Shoot(shooter, storage, RPM));
		}

		addCommands(toRun.toArray(Command[]::new));
    }
}
