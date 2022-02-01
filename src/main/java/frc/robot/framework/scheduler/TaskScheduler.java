package frc.robot.framework.scheduler;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.framework.scheduler.task.CommandTask;
import it.unimi.dsi.fastutil.ints.IntOpenHashSet;
import org.jetbrains.annotations.*;

import java.util.PriorityQueue;

public class TaskScheduler {
	private final DSControlWord controlWord = new DSControlWord();
	private final PriorityQueue<EnqueuedTask> queue = new PriorityQueue<>();
	private final IntOpenHashSet pendingCancellation = new IntOpenHashSet();
	private boolean stopped = false;

	private int currentTaskId = -1;
	private final long defaultPeriod;

	public TaskScheduler(long defaultPeriod) {
		this.defaultPeriod = defaultPeriod;
	}

	public TaskScheduler() {
		this(20);
	}

	public void start() {
		while (!stopped) {
			try {
				tick();
			} catch (Throwable e) {
				DriverStation.reportError("Encountered error while ticking scheduler!", e.getStackTrace());
			}
		}
	}

	public DSControlWord getControlWord() {
		return controlWord;
	}

	public RobotMode getCurrentMode() {
		controlWord.update();
		if (controlWord.isDisabled()) return RobotMode.DISABLED;
		if (controlWord.isAutonomous()) return RobotMode.AUTONOMOUS;
		if (controlWord.isTeleop()) return RobotMode.TELEOP;
		if (controlWord.isTest()) return RobotMode.TEST;

		throw new IllegalStateException("Unable to derive RobotMode from control word");
	}

	public void halt() {
		queue.clear();
		stopped = true;
	}

	public void tick() {
		EnqueuedTask next;
		//noinspection StatementWithEmptyBody – no logic needed; block until a task is available
		while ((next = queue.peek()) == null || next.getTargetTime() > System.currentTimeMillis()) {
		}

		if (next != queue.poll()) {
			throw new IllegalStateException("Mismatch in next task and next item in queue");
		}

		if (next.isPendingCancellation || pendingCancellation.contains(next.getId())) {
			pendingCancellation.remove(next.getId());
			return;
		}

		if (next.isPeriodic() && !next.getTask().isFinished()) {
			System.out.println("Periodic tick for " + next.getTask());
			next.tickPeriod();
			queue.add(next);
		}

		// Run task if it subscribes to the current mode
		if (!next.getTask().getDisallowedModes().contains(getCurrentMode())) {
			next.getTask().execute();
		} else {
			System.out.println("skipping task");
		}
	}

	private int nextTaskId() {
		currentTaskId++;
		return currentTaskId;
	}

	@Contract(value = "_ -> param1", mutates = "this")
	private EnqueuedTask queue(EnqueuedTask task) {
		System.out.printf("Enqueued %s%n", task.getTask());
		queue.add(task);
		return task;
	}

	@Contract(value = "_, _ -> new", mutates = "this")
	public EnqueuedTask queue(@NotNull Task task, long delay) {
		return queue(new EnqueuedTask(task, nextTaskId(), delay));
	}

	@Contract(value = "_ -> new", mutates = "this")
	public EnqueuedTask queue(@NotNull Task task) {
		return queue(new EnqueuedTask(task, nextTaskId()));
	}

	@Contract(value = "_, _, _ -> new", mutates = "this")
	public EnqueuedTask queuePeriodic(@NotNull Task task, long initialDelay, long period) {
		return queue(new EnqueuedTask(task, nextTaskId(), initialDelay, period));
	}

	@Contract(value = "_, _ -> new", mutates = "this")
	public EnqueuedTask queuePeriodic(@NotNull Task target, long period) {
		return queuePeriodic(target, period, period);
	}

	@Contract(value = "_ -> new", mutates = "this")
	public EnqueuedTask queuePeriodic(@NotNull Task target) {
		return queuePeriodic(target, defaultPeriod, defaultPeriod);
	}

	public EnqueuedTask scheduleCommand(Command command) {
		return queuePeriodic(new CommandTask(command));
	}

	public EnqueuedTask scheduleCommand(Command command, long period) {
		return queuePeriodic(new CommandTask(command), period);
	}

	public EnqueuedTask scheduleCommand(Command command, long initialDelay, long period) {
		return queuePeriodic(new CommandTask(command), initialDelay, period);
	}

	@Contract(mutates = "param1")
	public void cancel(@NotNull EnqueuedTask task) {
		task.isPendingCancellation = true;
	}

	@Contract(mutates = "this")
	public void cancel(int id) {
		pendingCancellation.add(id);
	}
}
