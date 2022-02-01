package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.framework.scheduler.ScheduledRobot;

public class Robot extends ScheduledRobot {
	public static void main(String[] args) {
		RobotBase.startRobot(Robot::new);
	}

	public Robot() {
		super(20);
	}

	@Override
	public void registerControls() {

	}

// needs to be populated
}
