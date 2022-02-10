package frc.robot.framework.scheduler;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.*;

public class SubsystemRegistry {

	private static final Set<Subsystem> subsystems = new HashSet<>();
	private static final Map<Subsystem, Command> requirementLocks = new HashMap<>();

	public static void register(Subsystem subsystem) {
		subsystems.add(subsystem);
	}

	public static Set<Subsystem> getEntries() {
		return subsystems;
	}

	public static void applyLock(Subsystem subsystem, Command command) {
		requirementLocks.put(subsystem, command);
	}

	public static boolean ownsLock(Subsystem subsystem, Command command) {
		return !requirementLocks.containsKey(subsystem) || requirementLocks.get(subsystem) == command;
	}

	public static Command getLockedCommand(Subsystem subsystem) {
		return requirementLocks.get(subsystem);
	}

	public static void releaseLock(Subsystem subsystem, Command command) {
		requirementLocks.remove(subsystem, command);
	}
}
