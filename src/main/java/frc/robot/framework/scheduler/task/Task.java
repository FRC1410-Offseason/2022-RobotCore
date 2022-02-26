package frc.robot.framework.scheduler.task;

import frc.robot.framework.scheduler.RobotMode;

import java.util.EnumSet;
import java.util.Set;

public abstract class Task {

    private boolean enabled;

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

	public abstract void execute();

    public void end() {}

	public boolean isFinished() {
		return false;
	}

    public boolean isValidToExecute() {
        return true;
    }

	public Set<RobotMode> getDisallowedModes() {
		return EnumSet.of(RobotMode.DISABLED);
	}
}
