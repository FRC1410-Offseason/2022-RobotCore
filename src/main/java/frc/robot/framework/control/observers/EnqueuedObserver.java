package frc.robot.framework.control.observers;

public class EnqueuedObserver implements Comparable<EnqueuedObserver>{

    private final Observer observer;
	private final int id;
	private final long period;
	boolean isPendingCancellation = false;
	private long targetTime;

    public EnqueuedObserver(Observer observer, int id, long initialDelay, long period) {
		this.observer = observer;
		this.id = id;
		this.targetTime = System.currentTimeMillis() + initialDelay;
		this.period = period;
	}

    public EnqueuedObserver(Observer observer, int id, long delay) {
		this(observer, id, delay, -1L);
	}

	public EnqueuedObserver(Observer observer, int id) {
		this(observer, id, 0L, -1L);
		this.targetTime = 0;
	}

    public Observer getObserver() {
        return observer;
    }

    public int getId() {
		return id;
	}

    public long getTargetTime() {
		return targetTime;
	}

    public long getPeriod() {
		return period;
	}

    public void tickPeriod() {
		this.targetTime = System.currentTimeMillis() + period;
	}

    @Override
    public int compareTo(EnqueuedObserver comparedObserver) {
        return Double.compare(this.targetTime, comparedObserver.targetTime);
    }
    
}
