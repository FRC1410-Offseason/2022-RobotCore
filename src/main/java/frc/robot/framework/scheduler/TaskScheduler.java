package frc.robot.framework.scheduler;

import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.framework.control.observers.*;
import frc.robot.framework.scheduler.task.CommandTask;
import frc.robot.framework.scheduler.task.Task;
import it.unimi.dsi.fastutil.ints.IntOpenHashSet;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.PriorityQueue;

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
            System.out.println("- Task: " + task.getTask() + " whose enabled state is: " + task.getTask().isEnabled());
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

	void clearButtonObservers() {
		for (var entry : observerQueue) {
			if (entry.getObserver() instanceof ButtonStateObserver) entry.getObserver().requestCancellation();
		}

		observerQueue.removeIf(entry -> entry.getObserver() instanceof ButtonStateObserver);
		debugDumpList();
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
            
            nextEnqueuedObserver.tickPeriod();
            observerQueue.add(nextEnqueuedObserver);
        }
        
        EnqueuedTask nextEnqueuedTask;
		//noinspection StatementWithEmptyBody – no logic needed; block until a task is available
		while ((nextEnqueuedTask = taskQueue.peek()) == null || nextEnqueuedTask.getTargetTime() > System.currentTimeMillis()) {}

        //Ensure task system state
		if (nextEnqueuedTask != taskQueue.poll()) {
			throw new IllegalStateException("Mismatch in next task and next item in taskQueue");
		}

        Task nextTask = nextEnqueuedTask.getTask();

        //Check if pending initialization, and add it
        if (nextTask.isRequestingExecution() && !nextTask.isEnabled() && nextTask.isValidToExecute()) {
            nextTask.enable();
            nextTask.removeRequestExecution();

            nextTask.initialize();
        }

        //Update time state for tasks and re-queue them to lists 
		if (nextEnqueuedTask.isPeriodic() && !nextTask.isFinished()) {
			nextEnqueuedTask.tickPeriod();
			taskQueue.add(nextEnqueuedTask);
		}

        if (nextTask.isRequestingCancellation() && nextTask.isEnabled()) {
            nextTask.interrupt();
            nextTask.disable();
            nextTask.removeRequestCancellation();
        }

		// Run task if it subscribes to the current mode
		if (nextTask.isEnabled() && !nextTask.getDisallowedModes().contains(getCurrentMode())) {
			nextTask.execute();
		}

        if (nextTask.isFinished() && nextTask.isEnabled()) {
            nextTask.end();
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
	public Task queuePeriodic(@NotNull Task task, long initialDelay, long period) {
        queueTask(new EnqueuedTask(task, nextTaskId(), initialDelay, period));
        return task;
	}

	@Contract(value = "_, _ -> new", mutates = "this")
	public Task queuePeriodic(@NotNull Task task, long period) {
        queuePeriodic(task, period, period);
        return task;
	}

	@Contract(value = "_ -> new", mutates = "this")
	public Task queuePeriodic(@NotNull Task task) {
        queuePeriodic(task, defaultPeriod, defaultPeriod);
        return task;
	}

    @Contract(value = "_, _, _ -> new", mutates = "this")
	public Observer queObservedTask(@NotNull Task task, @NotNull Observer observer, long initialDelay, long period) {
        observer.bind(task);
        queueTask(new EnqueuedTask(task, nextTaskId(), initialDelay, period));
        queueObserver(new EnqueuedObserver(observer, nextObserverId(), initialDelay, period));
		return observer;
	}

	@Contract(value = "_, _ -> new", mutates = "this")
	public Observer queueObservedTask(@NotNull Task task, @NotNull Observer observer, long period) {
		observer.bind(task);
        queuePeriodic(task, period, period);
        queueObserver(new EnqueuedObserver(observer, nextObserverId(), period));
        return observer;
	}

	@Contract(value = "_ -> new", mutates = "this")
	public Observer queueObservedTask(@NotNull Task task, @NotNull Observer observer) {
		observer.bind(task);
        queuePeriodic(task, defaultPeriod, defaultPeriod);
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

    public CommandTask scheduleDefaultCommand(Command command) {
		final CommandTask task = scheduleCommand(command);
        final DefaultCommandObserver observer = new DefaultCommandObserver();
        observer.bind(task);
        queueObserver(new EnqueuedObserver(observer, nextObserverId()));
        return task;
	}

    public CommandTask scheduleDefaultCommand(Command command, long period) {
		final CommandTask task = scheduleCommand(command, period);
        final DefaultCommandObserver observer = new DefaultCommandObserver();
        observer.bind(task);
        queueObserver(new EnqueuedObserver(observer, nextObserverId()));
        return task;
	}

    public CommandTask scheduleDefaultCommand(Command command, long initialDelay, long period) {
		final CommandTask task = scheduleCommand(command, initialDelay, period);
        final DefaultCommandObserver observer = new DefaultCommandObserver();
        observer.bind(task);
        queueObserver(new EnqueuedObserver(observer, nextObserverId()));
        return task;
	}

	private CommandTask scheduleCommand(Command command) {
		CommandTask localTask = new CommandTask(command);
        queuePeriodic(localTask);
        return localTask;
	}

	private CommandTask scheduleCommand(Command command, long period) {
		CommandTask localTask = new CommandTask(command);
        queuePeriodic(localTask, period);
        return localTask;
	}

	private CommandTask scheduleCommand(Command command, long initialDelay, long period) {
		CommandTask localTask = new CommandTask(command);
        queuePeriodic(localTask, initialDelay, period);
        return localTask;
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
