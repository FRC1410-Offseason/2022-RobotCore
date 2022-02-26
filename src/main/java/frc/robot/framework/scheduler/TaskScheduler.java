package frc.robot.framework.scheduler;

import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.framework.control.observers.DefaultCommandObserver;
import frc.robot.framework.control.observers.EnqueuedObserver;
import frc.robot.framework.control.observers.Observer;
import frc.robot.framework.scheduler.task.CommandTask;
import frc.robot.framework.scheduler.task.Task;
import it.unimi.dsi.fastutil.ints.IntOpenHashSet;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.PriorityQueue;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

public class TaskScheduler {

	private final DSControlWord controlWord = new DSControlWord();

    private final PriorityQueue<EnqueuedObserver> observerQueue = new PriorityQueue<>();

	private final PriorityQueue<EnqueuedTask> taskQueue = new PriorityQueue<>();
    
	private final IntOpenHashSet pendingCancellation = new IntOpenHashSet();

	private final long defaultPeriod;
	private final int m_notifier = NotifierJNI.initializeNotifier();
	private boolean stopped = false;

	private int currentTaskId = -1;
    private int currentObserverId = -1;

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

    public void debugDumpList() {
        System.out.println("Current Control Word: " + controlWord);

        System.out.println("Dumping Observer Queue:");
        for (EnqueuedObserver observer : observerQueue) {
            System.out.println("- Observer: " + observer + " with priority: " + observer.getObserver().getPriority());
        }

        for (EnqueuedTask task : taskQueue) {
            System.out.println("- Task: " + task.getTask() + " whose enabled state is: " + task.isEnabled());
        }
    }

	public void start() {

        if (dumpingDebugTelemetry) {
            debugDumpList();
        }

		while (!stopped) {
			try {
				tick();

				// Feed the watchdog
				NotifierJNI.updateNotifierAlarm(m_notifier, (long) (1e6));
			} catch (Throwable e) {
                System.out.println("Error encountered: " + e);
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

        //Iterate until found observer
        EnqueuedObserver nextEnqueuedObserver;
		if (!observerQueue.isEmpty()) {
            while ((nextEnqueuedObserver = observerQueue.peek()) == null) {}

            //Verify matching system states
            if (nextEnqueuedObserver != observerQueue.poll()) {
                throw new IllegalStateException("Mismatch in next observer and next item in observerQueue");
            }

            Observer nextObserver = nextEnqueuedObserver.getObserver();

            //Check state and interrupt commands if necessary and possible
            nextObserver.check();
        }
        
        EnqueuedTask nextTask;
		//noinspection StatementWithEmptyBody – no logic needed; block until a task is available
		while ((nextTask = taskQueue.peek()) == null || nextTask.getTargetTime() > System.currentTimeMillis()) {}

        //Ensure task system state
		if (nextTask != taskQueue.poll()) {
			throw new IllegalStateException("Mismatch in next task and next item in taskQueue");
		}

        //Remove tasks configured by scheduler.cancel()
		if (nextTask.isPendingCancellation || pendingCancellation.contains(nextTask.getId())) {
			pendingCancellation.remove(nextTask.getId());
			return;
		}

        //Check if pending initialization, and add it
        if (nextTask.isRequestingExecution() && nextTask.getTask().isValidToExecute()) {
            nextTask.enable();
            nextTask.removeRequestExecution();
        }

        //Update time state for tasks and re-queue them to lists 
		if (nextTask.isPeriodic() && !nextTask.getTask().isFinished()) {
			nextTask.tickPeriod();
			taskQueue.add(nextTask);
		}

		// Run task if it subscribes to the current mode
		if (nextTask.isEnabled() && !nextTask.getTask().getDisallowedModes().contains(getCurrentMode())) {
			nextTask.getTask().execute();
		}

        if (nextTask.isRequestingCancellation() && nextTask.isEnabled()) {
            nextTask.getTask().end();
            nextTask.disable();
            nextTask.removeRequestCancellation();
        }

        if (nextTask.getTask().isFinished() && nextTask.isEnabled()) {
            nextTask.getTask().end();
            nextTask.disable();
        }
	}

	private int nextTaskId() {
		currentTaskId++;
		return currentTaskId;
	}

    private int nextObserverId() {
		currentObserverId++;
		return currentObserverId;
	}

	@Contract(value = "_ -> param1", mutates = "this")
	private EnqueuedTask queueTask(EnqueuedTask task) {
		taskQueue.add(task);
		return task;
	}

    @Contract(value = "_ -> param1", mutates = "this")
	private EnqueuedObserver queueObserver(EnqueuedObserver observer) {
		observerQueue.add(observer);
		return observer;
	}

	@Contract(value = "_, _ -> new", mutates = "this")
	public EnqueuedTask queueTask(@NotNull Task task, long delay) {
		return queueTask(new EnqueuedTask(task, nextTaskId(), delay));
	}

	@Contract(value = "_ -> new", mutates = "this")
	public EnqueuedTask queueTask(@NotNull Task task) {
		return queueTask(new EnqueuedTask(task, nextTaskId()));
	}

	@Contract(value = "_, _, _ -> new", mutates = "this")
	public EnqueuedTask queuePeriodic(@NotNull Task task, long initialDelay, long period) {
		return queueTask(new EnqueuedTask(task, nextTaskId(), initialDelay, period));
	}

	@Contract(value = "_, _ -> new", mutates = "this")
	public EnqueuedTask queuePeriodic(@NotNull Task target, long period) {
		return queuePeriodic(target, period, period);
	}

	@Contract(value = "_ -> new", mutates = "this")
	public EnqueuedTask queuePeriodic(@NotNull Task target) {
		return queuePeriodic(target, defaultPeriod, defaultPeriod);
	}

    @Contract(value = "_, _, _ -> new", mutates = "this")
	public Observer queObservedTask(@NotNull Task task, @NotNull Observer observer, long initialDelay, long period) {
        observer.bind(queueTask(new EnqueuedTask(task, nextTaskId(), initialDelay, period)));
        queueObserver(new EnqueuedObserver(observer, nextObserverId(), initialDelay, period));
		return observer;
	}

	@Contract(value = "_, _ -> new", mutates = "this")
	public Observer queueObservedTask(@NotNull Task target, @NotNull Observer observer, long period) {
		observer.bind(queuePeriodic(target, period, period));
        queueObserver(new EnqueuedObserver(observer, nextObserverId(), period));
        return observer;
	}

	@Contract(value = "_ -> new", mutates = "this")
	public Observer queueObservedTask(@NotNull Task target, @NotNull Observer observer) {
		observer.bind(queuePeriodic(target, defaultPeriod, defaultPeriod));
        queueObserver(new EnqueuedObserver(observer, nextObserverId()));
        return observer;
	}

    @Contract(value = "_, _, _ -> new", mutates = "this")
	public Observer queueObservedCommand(@NotNull Command command, @NotNull Observer observer, long initialDelay, long period) {
        observer.bind(scheduleCommand(command, initialDelay, period));
        queueObserver(new EnqueuedObserver(observer, nextObserverId(), initialDelay, period));
		return observer;
	}

	@Contract(value = "_, _ -> new", mutates = "this")
	public Observer queueObservedCommand(@NotNull Command command, @NotNull Observer observer, long period) {
		observer.bind(scheduleCommand(command, period));
        queueObserver(new EnqueuedObserver(observer, nextObserverId(), period));
		return observer;
	}

	@Contract(value = "_ -> new", mutates = "this")
	public Observer queueObservedCommand(@NotNull Command command, @NotNull Observer observer) {
		observer.bind(scheduleCommand(command));
        queueObserver(new EnqueuedObserver(observer, nextObserverId()));
		return observer;
	}

    public EnqueuedTask scheduleDefaultCommand(Command command) {
		final EnqueuedTask task = scheduleCommand(command);
        final DefaultCommandObserver observer = new DefaultCommandObserver();
        observer.bind(task);
        queueObserver(new EnqueuedObserver(new DefaultCommandObserver(), nextObserverId()));
        return task;
	}

    public EnqueuedTask scheduleDefaultCommand(Command command, long period) {
		final EnqueuedTask task = scheduleCommand(command, period);
        final DefaultCommandObserver observer = new DefaultCommandObserver();
        observer.bind(task);
        queueObserver(new EnqueuedObserver(new DefaultCommandObserver(), nextObserverId()));
        return task;
	}

    public EnqueuedTask scheduleDefaultCommand(Command command, long initialDelay, long period) {
		final EnqueuedTask task = scheduleCommand(command, initialDelay, period);
        final DefaultCommandObserver observer = new DefaultCommandObserver();
        observer.bind(task);
        queueObserver(new EnqueuedObserver(new DefaultCommandObserver(), nextObserverId()));
        return task;
	}

	private EnqueuedTask scheduleCommand(Command command) {
		return queuePeriodic(new CommandTask(command));
	}

	private EnqueuedTask scheduleCommand(Command command, long period) {
		return queuePeriodic(new CommandTask(command), period);
	}

	private EnqueuedTask scheduleCommand(Command command, long initialDelay, long period) {
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
