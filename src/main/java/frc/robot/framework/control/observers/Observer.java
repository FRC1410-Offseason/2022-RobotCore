package frc.robot.framework.control.observers;

import java.util.EnumSet;
import java.util.List;
import java.util.Set;

import frc.robot.framework.scheduler.EnqueuedTask;
import frc.robot.framework.scheduler.RobotMode;
import frc.robotmap.IDs.SCHEDULER_PRIORITY;

public abstract class Observer {

    protected List<EnqueuedTask> boundTaskList = null;
    protected SCHEDULER_PRIORITY priority = SCHEDULER_PRIORITY.NULL;

    private boolean requesting = false;
    private boolean cancelling = false;

    public void bind(EnqueuedTask task) {
        boundTaskList.add(task);
        task.setPriority(priority);
    }

    public void unbind(EnqueuedTask task) {
        boundTaskList.remove(task);
        task.setPriority(SCHEDULER_PRIORITY.NULL);
    }

    public List<EnqueuedTask> getEnqueuedTasks() {
        return boundTaskList;
    }

    public void configurePriority(SCHEDULER_PRIORITY priority) {
        this.priority = priority;
    }

    public SCHEDULER_PRIORITY getPriority() {
        return priority;
    }

    public abstract void check();

    public void requestExecution() {
        for (EnqueuedTask task : boundTaskList) task.requestExecution();
    }

    public void removeRequestExecution() {
        for (EnqueuedTask task : boundTaskList) task.removeRequestExecution();
    }

    public void requestCancellation() {
        for (EnqueuedTask task : boundTaskList) task.requestCancellation();
    }

    public void removeRequestCancellation() {
        for (EnqueuedTask task : boundTaskList) task.removeRequestCancellation();
    }

    Set<RobotMode> getDisallowedModes() {
		return EnumSet.of(RobotMode.DISABLED);
	}
}
