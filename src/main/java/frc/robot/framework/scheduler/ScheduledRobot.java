// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.scheduler;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.*;

import frc.robot.framework.control.Axis;
import frc.robot.framework.control.ControlScheme;
import frc.robot.framework.scheduler.task.SubsystemPeriodicTask;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.*;

public abstract class ScheduledRobot extends RobotBase {

	protected final TaskScheduler scheduler;

	public ScheduledRobot(long defaultPeriod) {
		this.scheduler = new TaskScheduler(defaultPeriod);
	}

	@Override
	public final void startCompetition() {
		scheduler.queuePeriodic(new GameModeObserverTask());
		scheduler.queuePeriodic(new SubsystemPeriodicTask());

		robotInit();
		if (!RobotBase.isReal()) {
			simulationInit();
		}

        System.out.println("WE ARE PAST THE INITIALIZATION OHASE LMAO");

        if (this instanceof ControlScheme) {
            ((ControlScheme) this).registerControls();
        }
		scheduler.start();

        System.out.println("MUFUCKEN UHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");

        HAL.observeUserProgramStarting();
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

	private final class GameModeObserverTask implements Task {
		private RobotMode currentMode = null;

		@Override
		public void execute() {
			DSControlWord controlWord = scheduler.getControlWord();
			controlWord.update();

			if (controlWord.isEStopped()) {
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
