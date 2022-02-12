package frc.robot.framework.scheduler;

import java.util.EnumSet;
import java.util.Set;

import frc.robotmap.IDs.OBSERVER_PRIORITY;

public abstract class Observer implements Comparable<Observer> {

    protected EnqueuedTask task = null;
    protected OBSERVER_PRIORITY priority = OBSERVER_PRIORITY.NULL;

    public void bind(EnqueuedTask task) {
        this.task = task;
    }

    public void swtPriority(OBSERVER_PRIORITY priority) {
        this.priority = priority;
    }

    public EnqueuedTask getTask() {
        return task;
    }

    public OBSERVER_PRIORITY getPriority() {
        return priority;
    }

    public abstract void check();

    Set<RobotMode> getDisallowedModes() {
		return EnumSet.of(RobotMode.DISABLED);
	}

    @Override
	public int compareTo(Observer comparedObserver) {
		return Double.compare(priority.getId(), comparedObserver.getPriority().getId());
	}
}
