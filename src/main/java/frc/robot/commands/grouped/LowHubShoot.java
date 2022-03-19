package frc.robot.commands.grouped;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.commands.actions.*;
import frc.robot.subsystems.*;

import static frc.robotmap.Constants.*;
import static frc.robotmap.Tuning.*;

import java.util.ArrayList;

public class LowHubShoot extends SequentialCommandGroup {

	public LowHubShoot(Shooter shooter, ShooterArm shooterArm, Storage storage, LEDs leds, NetworkTableEntry RPM) {

		ArrayList<Command> toRun = new ArrayList<>();

		toRun.add(new ParallelCommandGroup(
			new InstantCommand(() -> leds.setProfile(LEDs.LEDProfile.GREEN)),
			new RunStorageForTime(storage, STORAGE_REVERSE_TIME, STORAGE_REVERSE_SPEED),
			new ShooterSpinup(shooter, RPM.getDouble(SHOOTER_LOW_HUB_RPM))
		));
		toRun.add(new Shoot(shooter, storage));
		toRun.add(new InstantCommand(() -> leds.setProfile(LEDs.LEDProfile.OFF)));

		addCommands(toRun.toArray(Command[]::new));
	}
}
