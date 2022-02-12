package frc.robot.framework.scheduler;

import java.util.EnumSet;
import java.util.Set;

public abstract class Observer {

    protected EnqueuedTask task = null;

    public void bind(EnqueuedTask task) {
        this.task = task;
    }

    public EnqueuedTask getTask() {
        return task;
    }

    public abstract void check();

    Set<RobotMode> getDisallowedModes() {
		return EnumSet.of(RobotMode.DISABLED);
	}
}
