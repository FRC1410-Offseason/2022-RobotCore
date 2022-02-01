package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.framework.scheduler.ScheduledRobot;

public class Robot extends ScheduledRobot {
	public static void main(String[] args) {
		RobotBase.startRobot(Robot::new);
	}

    @Override
    public void registerControls() {
    }

	private Robot() {
		super(20);
	}

	private final Timer timer = new Timer();
	private int ticks = 0;

	@Override
	public void robotInit() {
		timer.start();
		scheduler.queuePeriodic(() -> {
			ticks++;

			if (timer.get() >= 5) {
				timer.reset();
				System.out.println(ticks);
				ticks = 0;
			}
		}, 2);
	}
}
