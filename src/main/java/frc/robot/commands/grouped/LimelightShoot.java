package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.*;
import frc.robot.subsystems.*;

import java.util.ArrayList;

public class LimelightShoot extends SequentialCommandGroup {

    public LimelightShoot(Drivetrain drivetrain, Limelight limelight, Shooter shooter, Storage storage, ShooterArm shooterArm) {
        int RPM = (int) shooter.targetRPM(1);// Make real value

		ArrayList<Command> toRun = new ArrayList<>();

		toRun.add(new LimelightAnglePID(limelight, drivetrain));

		if (shooter.isOuttakeQueued()) {
			toRun.add(new Shoot(shooter, shooterArm, storage, RPM, 1));
			toRun.add(new ShootOuttake(shooter, shooterArm, storage));
		} else {
			toRun.add(new Shoot(shooter, shooterArm, storage, RPM, storage.getCurrentState().getNumCargo()));
		}

		addCommands(toRun.toArray(Command[]::new));
    }
}
