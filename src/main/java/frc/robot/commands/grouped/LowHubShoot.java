package frc.robot.commands.grouped;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.*;
import frc.robot.subsystems.*;

import static frc.robotmap.Constants.*;

import java.util.ArrayList;

public class LowHubShoot extends SequentialCommandGroup {

	public LowHubShoot(Shooter shooter, ShooterArm shooterArm, Storage storage, NetworkTableEntry RPM) {
		// TODO: Add calculation for desired exit velocity of cargo

		ArrayList<Command> toRun = new ArrayList<>();

		toRun.add(new ParallelCommandGroup(
			new RunStorageForTime(storage, STORAGE_REVERSE_TIME, STORAGE_REVERSE_SPEED),
			new ShooterSpinup(shooter, RPM.getDouble(0))
		));
		toRun.add(new Shoot(shooter, storage));

		addCommands(toRun.toArray(Command[]::new));
	}
}
