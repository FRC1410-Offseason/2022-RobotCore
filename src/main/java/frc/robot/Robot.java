package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.framework.control.ControlScheme;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.framework.scheduler.TaskScheduler;

public class Robot extends ScheduledRobot implements ControlScheme {
	public static void main(String[] args) {
		RobotBase.startRobot(Robot::new);
	}

	public Robot() {
		super(20);
	}

    @Override
    public TaskScheduler getScheduler() {
        return scheduler;
    }

	@Override
	public void registerControls() {

	}
// needs to be populated
}
