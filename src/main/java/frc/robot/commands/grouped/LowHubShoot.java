package frc.robot.commands.grouped;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robotmap.Constants.*;
import frc.robot.commands.actions.*;
import frc.robot.subsystems.*;

import java.util.ArrayList;

public class LowHubShoot extends SequentialCommandGroup {

	public LowHubShoot(Shooter shooter, ShooterArm shooterArm, Storage storage, double RPM) {
		// TODO: Add calculation for desired exit velocity of cargo

		ArrayList<Command> toRun = new ArrayList<>();

		toRun.add(new ParallelCommandGroup(
			new ShooterSpinup(shooter, RPM),
			new SetShooterArmAngle(shooterArm, SHOOTER_ARM_MAX_ANGLE),
			new RunStorageForTime(storage, 0.1, -0.5)));
		toRun.add(new Shoot(shooter, storage));

		addCommands(toRun.toArray(Command[]::new));
	}
}
