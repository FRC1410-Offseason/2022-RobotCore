package frc.robot.framework.scheduler;

import java.util.*;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class SubsystemRegistry {

	private static final Set<Subsystem> subsystems = new HashSet<>();
	private static final Map<Subsystem, EnqueuedTask> requirementLocks = new HashMap<>();

	public static void register(Subsystem subsystem) {
		subsystems.add(subsystem);
	}

	public static Set<Subsystem> getEntries() {
		return subsystems;
	}

	public static void applyLock(Subsystem subsystem, EnqueuedTask task) {
		requirementLocks.put(subsystem, task);
	}

	public static boolean ownsLock(Subsystem subsystem, EnqueuedTask task) {
		return !requirementLocks.containsKey(subsystem) || requirementLocks.get(subsystem) == task;
	}

	public static EnqueuedTask getLockingTask(Subsystem subsystem) {
		return requirementLocks.get(subsystem);
	}

	public static void releaseLock(Subsystem subsystem, EnqueuedTask task) {
		requirementLocks.remove(subsystem, task);
	}
}
