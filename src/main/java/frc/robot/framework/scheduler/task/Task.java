package frc.robot.framework.scheduler.task;

import frc.robot.framework.scheduler.RobotMode;
import frc.robotmap.IDs.SchedulerPriority;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public abstract class Task {

	private final Set<RobotMode> disallowedModes = new HashSet<>();
	private SchedulerPriority priority = SchedulerPriority.NULL;

    private boolean requesting = false;
    private boolean cancelling = false;

    private boolean enabled = false;

	public Task() {
		disallowedModes.add(RobotMode.DISABLED);
	}

	public void setPriority(SchedulerPriority priority) {
        this.priority = priority;
    }

    public SchedulerPriority getPriority() {
        return priority;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void requestExecution() {
        requesting = true;
    }

    public void removeRequestExecution() {
        requesting = false;
    }

    public boolean isRequestingExecution() {
        return requesting;
    }

    public void requestCancellation() {
        cancelling = true;
    }

    public void removeRequestCancellation() {
        cancelling = false;
    }

    public boolean isRequestingCancellation() {
        return cancelling;
    }

    public void initialize() {}
    
    public abstract void execute();

    public void end() {}

    public void interrupt() {}

	public boolean isFinished() {
        return false;
    }

    public boolean isValidToExecute() {
        return true;
    }

	public Task disallowModes(RobotMode... modes) {
		Collections.addAll(disallowedModes, modes);
		return this;
	}

	public Task removedDisallowedModes(RobotMode... modes) {
		for (var mode : modes) {
			disallowedModes.remove(mode);
		}
		return this;
	}

	public Set<RobotMode> getDisallowedModes() {
		return disallowedModes;
	}
}
