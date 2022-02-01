package frc.robot.framework.scheduler;

import edu.wpi.first.hal.HAL;

public enum RobotMode {
	DISABLED {
		@Override
		public void enter(ScheduledRobot robot) {
			robot.disabledInit();
		}

		@Override
		public void periodic(ScheduledRobot robot) {
			HAL.observeUserProgramDisabled();
			robot.disabledPeriodic();
		}

		@Override
		public void exit(ScheduledRobot robot) {
			robot.disabledExit();
		}
	},

	AUTONOMOUS {
		@Override
		public void enter(ScheduledRobot robot) {
			robot.autonomousInit();
		}

		@Override
		public void periodic(ScheduledRobot robot) {
			HAL.observeUserProgramAutonomous();
			robot.autonomousPeriodic();
		}

		@Override
		public void exit(ScheduledRobot robot) {
			robot.autonomousExit();
		}
	},

	TELEOP {
		@Override
		public void enter(ScheduledRobot robot) {
			robot.teleopInit();
		}

		@Override
		public void periodic(ScheduledRobot robot) {
			HAL.observeUserProgramTeleop();
			robot.teleopPeriodic();
		}

		@Override
		public void exit(ScheduledRobot robot) {
			robot.teleopExit();
		}
	},

	TEST {
		@Override
		public void enter(ScheduledRobot robot) {
			robot.testInit();
		}

		@Override
		public void periodic(ScheduledRobot robot) {
			HAL.observeUserProgramTest();
			robot.testPeriodic();
		}

		@Override
		public void exit(ScheduledRobot robot) {
			robot.testExit();
		}
	};

	public abstract void enter(ScheduledRobot robot);

	public abstract void periodic(ScheduledRobot robot);

	public abstract void exit(ScheduledRobot robot);
}
