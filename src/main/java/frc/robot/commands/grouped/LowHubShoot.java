package frc.robot.commands.grouped;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.*;
import frc.robot.subsystems.*;

import static frc.robotmap.Constants.*;
import static frc.robotmap.Tuning.*;

import java.util.ArrayList;

public class LowHubShoot extends SequentialCommandGroup {

	public LowHubShoot(Shooter shooter, ShooterArm shooterArm, Storage storage, NetworkTableEntry RPM) {
		// TODO: Add calculation for desired exit velocity of cargo

		ArrayList<Command> toRun = new ArrayList<>();

		toRun.add(new RunStorageForTime(storage, STORAGE_REVERSE_TIME, STORAGE_REVERSE_SPEED));
        toRun.add(new ShooterSpinup(shooter, RPM.getDouble(SHOOTER_LOW_HUB_RPM)));
		toRun.add(new Shoot(shooter, storage));

		addCommands(toRun.toArray(Command[]::new));
	}
}
