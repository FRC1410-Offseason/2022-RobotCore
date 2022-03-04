// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.scheduler;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.framework.control.controllers.ControlScheme;
import frc.robot.framework.scheduler.task.SubsystemPeriodicTask;
import frc.robot.framework.scheduler.task.Task;

public abstract class ScheduledRobot extends RobotBase implements ControlScheme {

	protected final TaskScheduler scheduler;

	public ScheduledRobot(long defaultPeriod) {
		this.scheduler = new TaskScheduler(defaultPeriod);
	}

	@Override
	public TaskScheduler getScheduler() {
		return scheduler;
	}

	@Override
	public final void startCompetition() {
		scheduler.queuePeriodic(new GameModeObserverTask().removedDisallowedModes(RobotMode.DISABLED)).enable();
		scheduler.queuePeriodic(new SubsystemPeriodicTask()).enable();

		robotInit();
		if (!RobotBase.isReal()) {
			simulationInit();
		}

		HAL.observeUserProgramStarting();
		registerControls();

		scheduler.start();
	}

	@Override
	public void endCompetition() {

	}

	public void robotInit() {

	}

	public void simulationInit() {

	}

	public void disabledInit() {

	}

	public void autonomousInit() {

	}

	public void teleopInit() {

	}

	public void testInit() {

	}

	public void simulationPeriodic() {


	}

	public void disabledPeriodic() {

	}

	public void autonomousPeriodic() {

	}

	public void teleopPeriodic() {

	}


	public void testPeriodic() {

	}

	public void disabledExit() {

	}

	public void autonomousExit() {

	}

	public void teleopExit() {

	}

	public void testExit() {

	}

	public final class GameModeObserverTask extends Task {

		private RobotMode currentMode = null;

		@Override
		public void execute() {
			DSControlWord controlWord = scheduler.getControlWord();
			controlWord.update();

			if (controlWord.isEStopped()) {
				scheduler.interruptAll();
				scheduler.halt();

				throw new IllegalStateException("Emergency stopped! All functionality has been halted.");
			}

			RobotMode mode;
			if (controlWord.isDisabled()) {
				mode = RobotMode.DISABLED;
			} else if (controlWord.isAutonomous()) {
				mode = RobotMode.AUTONOMOUS;
			} else if (controlWord.isTeleop()) {
				mode = RobotMode.TELEOP;
			} else if (controlWord.isTest()) {
				mode = RobotMode.TEST;
			} else {
				throw new IllegalStateException("Unexpected game state! Cannot find a matching mode.");
			}

			// Indicates that a mode change happened
			if (currentMode != mode) {
				if (currentMode != null) currentMode.exit(ScheduledRobot.this);
				mode.enter(ScheduledRobot.this);

				currentMode = mode;
			} else {
				mode.periodic(ScheduledRobot.this);
			}

			if (RobotBase.isSimulation()) {
				ScheduledRobot.this.simulationPeriodic();
			}
		}
	}
}
