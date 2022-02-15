package frc.robot.framework.control.observers;

import java.util.EnumSet;
import java.util.Set;

import frc.robot.framework.scheduler.EnqueuedTask;
import frc.robot.framework.scheduler.RobotMode;
import frc.robotmap.IDs.SCHEDULER_PRIORITY;

public abstract class Observer implements Comparable<Observer> {

    protected EnqueuedTask task = null;
    protected SCHEDULER_PRIORITY priority = SCHEDULER_PRIORITY.NULL;

    private boolean requesting = false;
    private boolean cancelling = false;

    public void bind(EnqueuedTask task) {
        this.task = task;
        task.setPriority(priority);
    }

    public EnqueuedTask getEnqueuedTask() {
        return task;
    }

    public void setPriority(SCHEDULER_PRIORITY priority) {
        this.priority = priority;
    }

    public SCHEDULER_PRIORITY getPriority() {
        return priority;
    }

    public void requestExecution() {
        requesting = true;
    }

    public void removeRequestExecution() {
        requesting = true;
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

    public abstract void check();

    Set<RobotMode> getDisallowedModes() {
		return EnumSet.of(RobotMode.DISABLED);
	}

    @Override
	public int compareTo(Observer comparedObserver) {
		return Double.compare(priority.getId(), comparedObserver.getPriority().getId());
	}
}
