package frc.robot.framework.scheduler;

import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.*;

public class SubsystemRegistry {
	private static final Set<Subsystem> subsystems = new HashSet<>();

	public static void register(Subsystem subsystem) {
		subsystems.add(subsystem);
	}

	public static Set<Subsystem> getEntries() {
		return subsystems;
	}
}
