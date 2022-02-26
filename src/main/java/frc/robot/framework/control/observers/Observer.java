package frc.robot.framework.control.observers;

import java.util.EnumSet;
import java.util.HashSet;
import java.util.Set;

import frc.robot.framework.scheduler.RobotMode;
import frc.robot.framework.scheduler.task.Task;
import frc.robotmap.IDs.SCHEDULER_PRIORITY;

public abstract class Observer {

    protected Set<Task> boundTaskList = new HashSet<Task>();
    protected SCHEDULER_PRIORITY priority = SCHEDULER_PRIORITY.NULL;

    public void bind(Task task) {
        boundTaskList.add(task);
        task.setPriority(priority);
    }

    public void unbind(Task task) {
        boundTaskList.remove(task);
        task.setPriority(SCHEDULER_PRIORITY.NULL);
    }

    public Set<Task> getEnqueuedTasks() {
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
        for (Task task : boundTaskList) task.requestExecution();
    }

    public void removeRequestExecution() {
        for (Task task : boundTaskList) task.removeRequestExecution();
    }

    public void requestCancellation() {
        for (Task task : boundTaskList) task.requestCancellation();
    }

    public void removeRequestCancellation() {
        for (Task task : boundTaskList) task.removeRequestCancellation();
    }

    Set<RobotMode> getDisallowedModes() {
		return EnumSet.of(RobotMode.DISABLED);
	}
}
