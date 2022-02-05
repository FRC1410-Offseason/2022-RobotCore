package frc.robot.framework.scheduler;

import frc.robot.framework.scheduler.task.Task;
public class EnqueuedTask implements Comparable<EnqueuedTask> {
	private final Task task;
	private final int id;
	private long targetTime;
	private final long period;
	boolean isPendingCancellation = false;

	public EnqueuedTask(Task task, int id, long initialDelay, long period) {
		this.task = task;
		this.id = id;
		this.targetTime = System.currentTimeMillis() + initialDelay;
		this.period = period;
	}

	public EnqueuedTask(Task task, int id, long delay) {
		this(task, id, delay, -1L);
	}

	public EnqueuedTask(Task task, int id) {
		this(task, id, 0L, -1L);
		this.targetTime = 0;
	}

	public Task getTask() {
		return task;
	}

	public int getId() {
		return id;
	}

	public long getTargetTime() {
		return targetTime;
	}

	public boolean isPeriodic() {
		return period != -1L;
	}

	public long getPeriod() {
		return period;
	}

	public void tickPeriod() {
		if (!isPeriodic()) {
			throw new IllegalStateException("Cannot tick a non-periodic method");
		}

		this.targetTime = System.currentTimeMillis() + period;
	}

	@Override
	public int compareTo(EnqueuedTask o) {
		return Double.compare(this.targetTime, o.targetTime);
	}
}
