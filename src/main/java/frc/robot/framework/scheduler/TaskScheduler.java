package frc.robot.framework.scheduler;

import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.framework.control.observers.DefaultCommandObserver;
import frc.robot.framework.control.observers.EnqueuedObserver;
import frc.robot.framework.control.observers.Observer;
import frc.robot.framework.scheduler.task.CommandTask;
import frc.robot.framework.scheduler.task.Task;
import it.unimi.dsi.fastutil.ints.IntOpenHashSet;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.Map;
import java.util.PriorityQueue;

public class TaskScheduler {

	private final DSControlWord controlWord = new DSControlWord();

    private final PriorityQueue<EnqueuedObserver> observerQueue = new PriorityQueue<>();

	private final PriorityQueue<EnqueuedTask> taskQueue = new PriorityQueue<>();
    private final Map<EnqueuedTask, Subsystem> bindingSubsystemTasksToCancel = new HashMap<>();

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

        if (dumpingDebugTelemetry) debugDumpList();

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
		while ((nextEnqueuedObserver = observerQueue.peek()) == null) {}

        //Verify matching system states
		if (nextEnqueuedObserver != observerQueue.poll()) {
			throw new IllegalStateException("Mismatch in next observer and next item in observerQueue");
		}

        Observer nextObserver = nextEnqueuedObserver.getObserver();

        //Check state and interrupt commands if necessary and possible
        nextObserver.check();

        //Bless this mess
        if (nextObserver.isRequestingExecution()) {
            if (nextObserver.getEnqueuedTask().getTask() instanceof CommandTask) {
                Command requestedCommand = ((CommandTask) nextObserver.getEnqueuedTask().getTask()).getCommand();

                if (!requestedCommand.getRequirements().isEmpty()) {
                    boolean isHighestPriority = true;
                    for (Subsystem requirement : requestedCommand.getRequirements()) {
                        if (!(SubsystemRegistry.getLockingTask(requirement) == null)) {
                            if (nextObserver.getPriority().getValue() < SubsystemRegistry.getLockingTask(requirement).getPriority().getValue()) {
                                isHighestPriority = false;
                                break;
                            } else {
                                bindingSubsystemTasksToCancel.put(SubsystemRegistry.getLockingTask(requirement), requirement);
                            }
                        }
                    }
                    if (isHighestPriority) {    //Passed check
                        //Remove and interrupt currently running command
                        for (EnqueuedTask taskToCancel : bindingSubsystemTasksToCancel.keySet()) {
                            ((CommandTask) taskToCancel.getTask()).interrupt();
                            ((CommandTask) taskToCancel.getTask()).end();
                            taskToCancel.disable();
                            SubsystemRegistry.releaseLock(bindingSubsystemTasksToCancel.get(taskToCancel), taskToCancel);
                        }

                        //Add currently requested command
                        for (Subsystem requirement : requestedCommand.getRequirements()) {
                            SubsystemRegistry.applyLock(requirement, nextObserver.getEnqueuedTask());
                        }

                        if (dumpingDebugTelemetry) System.out.println("Succeeded interruption by " + ((CommandTask) nextObserver.getEnqueuedTask().getTask()).getCommand());
                        nextObserver.getEnqueuedTask().enable();
                        nextObserver.removeRequestExecution();
                    } else {
                        if (dumpingDebugTelemetry) System.out.println("Failed interruption by " + ((CommandTask) nextObserver.getEnqueuedTask().getTask()).getCommand());
                    }
                } else {
                    if (dumpingDebugTelemetry) System.out.println("Succeeded interruption by " + nextObserver.getEnqueuedTask().getTask() + " due to lack of subsystem requirements");
                    nextObserver.getEnqueuedTask().enable();
                    nextObserver.removeRequestExecution();
                }
            } else {
                if (dumpingDebugTelemetry) System.out.println("Succeeded interruption by " + nextObserver.getEnqueuedTask().getTask() + " due to not being a CommandTask");
                nextObserver.getEnqueuedTask().enable();
                nextObserver.removeRequestExecution();
            }
        }

        //Disable if requesting cancellation
        if (nextObserver.isRequestingCancellation()) {
            if (nextObserver.getEnqueuedTask().getTask() instanceof CommandTask) {
                for (Subsystem requirement : ((CommandTask) nextObserver.getEnqueuedTask().getTask()).getCommand().getRequirements()) {
                    SubsystemRegistry.releaseLock(requirement, nextObserver.getEnqueuedTask());
                }
            }
            nextObserver.getEnqueuedTask().disable();
            nextObserver.removeRequestCancellation();
        }

        //Tick target time and add them back to the queue
        nextEnqueuedObserver.tickPeriod();
        observerQueue.add(nextEnqueuedObserver);
        
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

        //Update time state for tasks and re-queue them to lists 
		if (nextTask.isPeriodic() && !nextTask.getTask().isFinished()) {
			nextTask.tickPeriod();
			taskQueue.add(nextTask);
		}

		// Run task if it subscribes to the current mode
		if (nextTask.isEnabled() && !nextTask.getTask().getDisallowedModes().contains(getCurrentMode())) {
			nextTask.getTask().execute();
		}

        if (nextTask.getTask().isFinished()) {
            if (nextTask.getTask() instanceof CommandTask) {
                for (Subsystem requirement : ((CommandTask) nextTask.getTask()).getCommand().getRequirements()) {
                    SubsystemRegistry.releaseLock(requirement, nextTask);
                }
            }
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
	public EnqueuedTask queObservedTask(@NotNull Task task, @NotNull Observer observer, long initialDelay, long period) {
        observer.bind(queueTask(new EnqueuedTask(task, nextTaskId(), initialDelay, period)));
        queueObserver(new EnqueuedObserver(observer, nextObserverId(), initialDelay, period));
		return observer.getEnqueuedTask();
	}

	@Contract(value = "_, _ -> new", mutates = "this")
	public EnqueuedTask queueObservedTask(@NotNull Task target, @NotNull Observer observer, long period) {
		observer.bind(queuePeriodic(target, period, period));
        queueObserver(new EnqueuedObserver(observer, nextObserverId(), period));
        return observer.getEnqueuedTask();
	}

	@Contract(value = "_ -> new", mutates = "this")
	public EnqueuedTask queueObservedTask(@NotNull Task target, @NotNull Observer observer) {
		observer.bind(queuePeriodic(target, defaultPeriod, defaultPeriod));
        queueObserver(new EnqueuedObserver(observer, nextObserverId()));
        return observer.getEnqueuedTask();
	}

    @Contract(value = "_, _, _ -> new", mutates = "this")
	public EnqueuedTask queueObservedCommand(@NotNull Command command, @NotNull Observer observer, long initialDelay, long period) {
        observer.bind(scheduleCommand(command, initialDelay, period));
        queueObserver(new EnqueuedObserver(observer, nextObserverId(), initialDelay, period));
		return observer.getEnqueuedTask();
	}

	@Contract(value = "_, _ -> new", mutates = "this")
	public EnqueuedTask queueObservedCommand(@NotNull Command command, @NotNull Observer observer, long period) {
		observer.bind(scheduleCommand(command, period));
        queueObserver(new EnqueuedObserver(observer, nextObserverId(), period));
		return observer.getEnqueuedTask();
	}

	@Contract(value = "_ -> new", mutates = "this")
	public EnqueuedTask queueObservedCommand(@NotNull Command command, @NotNull Observer observer) {
		observer.bind(scheduleCommand(command));
        queueObserver(new EnqueuedObserver(observer, nextObserverId()));
		return observer.getEnqueuedTask();
	}

    public EnqueuedTask scheduleDefaultCommand(Command command) {
		final Observer observer = new DefaultCommandObserver();
        observer.bind(scheduleCommand(command));
        queueObserver(new EnqueuedObserver(observer, nextObserverId()));
        return observer.getEnqueuedTask();
	}

    public EnqueuedTask scheduleDefaultCommand(Command command, long period) {
		final Observer observer = new DefaultCommandObserver();
        observer.bind(scheduleCommand(command, period));
        queueObserver(new EnqueuedObserver(observer, nextObserverId(), period));
        return observer.getEnqueuedTask();
	}

    public EnqueuedTask scheduleDefaultCommand(Command command, long initialDelay, long period) {
		final Observer observer = new DefaultCommandObserver();
        observer.bind(scheduleCommand(command, initialDelay, period));
        queueObserver(new EnqueuedObserver(observer, nextObserverId(), initialDelay, period));
        return observer.getEnqueuedTask();
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
