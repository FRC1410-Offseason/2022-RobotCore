package frc.robot.framework.scheduler;

import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.framework.scheduler.task.CommandTask;
import frc.robot.framework.scheduler.task.Task;
import it.unimi.dsi.fastutil.ints.IntOpenHashSet;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.PriorityQueue;

public class TaskScheduler {

	private final DSControlWord controlWord = new DSControlWord();

    private final PriorityQueue<Observer> observerQueue = new PriorityQueue<>();

	private final PriorityQueue<EnqueuedTask> taskQueue = new PriorityQueue<>();
	private final IntOpenHashSet pendingCancellation = new IntOpenHashSet();
    private final IntOpenHashSet pendingInitialization = new IntOpenHashSet();
	private final long defaultPeriod;
	private final int m_notifier = NotifierJNI.initializeNotifier();
	private boolean stopped = false;
	private int currentTaskId = -1;

    private boolean dumpingDebugTelemetry = false;

	public TaskScheduler(long defaultPeriod) {
		this.defaultPeriod = defaultPeriod;
	}

	public TaskScheduler() {
		this(20);
	}

    public void enableDebugTelemetry() {
        dumpingDebugTelemetry = true;
    }

    public void debugDump() {
        System.out.println("Current Control Word: " + controlWord);
        
        System.out.println("Dumpign Observer Queue:");
        for (Observer observer : observerQueue) {
            System.out.println("- Observer: " + observer);
        }

        for (EnqueuedTask task : taskQueue) {
            System.out.println("Task: " + task.getTask());
        }
    }

	public void start() {
		while (!stopped) {
			try {
				tick();

				// Feed the watchdog
				NotifierJNI.updateNotifierAlarm(m_notifier, (long) (1e6));
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
		taskQueue.clear();
		stopped = true;
	}

	public void tick() {

        if (dumpingDebugTelemetry) debugDump();

		EnqueuedTask next;
		//noinspection StatementWithEmptyBody – no logic needed; block until a task is available
		while ((next = taskQueue.peek()) == null || next.getTargetTime() > System.currentTimeMillis()) {
		}

		if (next != taskQueue.poll()) {
			throw new IllegalStateException("Mismatch in next task and next item in taskQueue");
		}

		if (next.isPendingCancellation || pendingCancellation.contains(next.getId())) {
			pendingCancellation.remove(next.getId());
			return;
		}

		if (next.isPeriodic() && !next.getTask().isFinished()) {
			next.tickPeriod();
			taskQueue.add(next);
		}

		// Run task if it subscribes to the current mode
		if (!next.getTask().getDisallowedModes().contains(getCurrentMode())) {
			next.getTask().execute();
		}
	}

	private int nextTaskId() {
		currentTaskId++;
		return currentTaskId;
	}

	@Contract(value = "_ -> param1", mutates = "this")
	private EnqueuedTask taskQueue(EnqueuedTask task) {
		taskQueue.add(task);
		return task;
	}

	@Contract(value = "_, _ -> new", mutates = "this")
	public EnqueuedTask taskQueue(@NotNull Task task, long delay) {
		return taskQueue(new EnqueuedTask(task, nextTaskId(), delay));
	}

	@Contract(value = "_ -> new", mutates = "this")
	public EnqueuedTask taskQueue(@NotNull Task task) {
		return taskQueue(new EnqueuedTask(task, nextTaskId()));
	}

	@Contract(value = "_, _, _ -> new", mutates = "this")
	public EnqueuedTask queuePeriodic(@NotNull Task task, long initialDelay, long period) {
		return taskQueue(new EnqueuedTask(task, nextTaskId(), initialDelay, period));
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
