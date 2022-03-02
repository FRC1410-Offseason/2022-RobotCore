package frc.robot.framework.subsystem;

import java.util.*;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.framework.scheduler.task.CommandTask;

public class SubsystemRegistry {

	private static final Set<Subsystem> subsystems = new HashSet<>();
	private static final Map<Subsystem, CommandTask> requirementLocks = new HashMap<>();

	public static void register(Subsystem subsystem) {
		subsystems.add(subsystem);
	}

	public static Set<Subsystem> getEntries() {
		return subsystems;
	}

    public static boolean isHighestPriority(CommandTask task) {
        boolean isHighestPriority = true;
        for (Subsystem requirement : task.getCommand().getRequirements()) {
            if (!(getLockingTask(requirement) == null)) {
                if (getLockingTask(requirement).getPriority().getValue() > task.getPriority().getValue()) isHighestPriority = false;
            }
        }
        return isHighestPriority;
    }

    public static void interruptAllNecessaryLocks(CommandTask task) {
        for (Subsystem requirement : task.getCommand().getRequirements()) {
            if (!(getLockingTask(requirement) == null)) {
                CommandTask interruptedTask = getLockingTask(requirement);
                interruptedTask.interrupt();
                interruptedTask.disable();
            }
        }
    }

    public static void applyAllNecessaryLocks(CommandTask task) {
        for (Subsystem requirement : task.getCommand().getRequirements()) {
            applyLock(requirement, task);
        }
    }

    public static void releaseAllNecessaryLocks(CommandTask task) {
        for (Subsystem requirement : task.getCommand().getRequirements()) {
            releaseLock(requirement, task);
        }
    }

	public static void applyLock(Subsystem subsystem, CommandTask task) {
        requirementLocks.put(subsystem, task);
	}

	public static CommandTask getLockingTask(Subsystem subsystem) {
		return requirementLocks.get(subsystem);
	}

	public static void releaseLock(Subsystem subsystem, CommandTask task) {
        requirementLocks.remove(subsystem, task);
	}
}
