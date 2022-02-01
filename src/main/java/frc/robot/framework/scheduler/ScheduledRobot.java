// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.framework.scheduler;

import edu.wpi.first.wpilibj.*;

import frc.robot.framework.scheduler.task.SubsystemPeriodicTask;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.framework.control.input.Axis;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.*;

public abstract class ScheduledRobot extends RobotBase {
    public XboxController DriverController = new XboxController(DRIVER_CONTROLLER_PORT);
	public XboxController OperatorController = new XboxController(OPERATOR_CONTROLLER_PORT);

	protected final Axis DriverLeftXAxis = new Axis(DriverController, AXIS_ID.LEFT_X, DRIVER_DEADZONE_VALUE);
	protected final Axis DriverRightXAxis = new Axis(DriverController, AXIS_ID.LEFT_Y, DRIVER_DEADZONE_VALUE);
	protected final Axis DriverLeftYAxis = new Axis(DriverController, AXIS_ID.RIGHT_X, DRIVER_DEADZONE_VALUE);
	protected final Axis DriverRightYAxis = new Axis(DriverController, AXIS_ID.RIGHT_Y, DRIVER_DEADZONE_VALUE);
	protected final Axis DriverLeftTrigger = new Axis(DriverController, AXIS_ID.LEFT_TRIGGER, DRIVER_DEADZONE_VALUE);
	protected final Axis DriverRightTrigger = new Axis(DriverController, AXIS_ID.RIGHT_TRIGGER, DRIVER_DEADZONE_VALUE);

	protected final Axis OperatorLeftXAxis = new Axis(OperatorController, AXIS_ID.LEFT_X, OPERATOR_DEADZONE_VALUE);
	protected final Axis OperatorRightXAxis = new Axis(OperatorController, AXIS_ID.RIGHT_X, OPERATOR_DEADZONE_VALUE);
	protected final Axis OperatorLeftYAxis = new Axis(OperatorController, AXIS_ID.LEFT_Y, OPERATOR_DEADZONE_VALUE);
	protected final Axis OperatorRightYAxis = new Axis(OperatorController, AXIS_ID.RIGHT_Y, OPERATOR_DEADZONE_VALUE);
	protected final Axis OperatorLeftTrigger = new Axis(OperatorController, AXIS_ID.LEFT_TRIGGER, OPERATOR_DEADZONE_VALUE);
	protected final Axis OperatorRightTrigger = new Axis(OperatorController, AXIS_ID.RIGHT_TRIGGER, OPERATOR_DEADZONE_VALUE);

	protected final JoystickButton DriverAButton = new JoystickButton(DriverController, A_BUTTON_ID);
	protected final JoystickButton DriverBButton = new JoystickButton(DriverController, B_BUTTON_ID);
	protected final JoystickButton DriverXButton = new JoystickButton(DriverController, X_BUTTON_ID);
	protected final JoystickButton DriverYButton = new JoystickButton(DriverController, Y_BUTTON_ID);
	protected final JoystickButton DriverLeftBumper = new JoystickButton(DriverController, LEFT_BUMPER_ID);
	protected final JoystickButton DriverRightBumper = new JoystickButton(DriverController, RIGHT_BUMPER_ID);
	protected final JoystickButton DriverLeftStickButton = new JoystickButton(DriverController, LEFT_STICK_BUTTON_ID);
	protected final JoystickButton DriverRightStickButton = new JoystickButton(DriverController, RIGHT_STICK_BUTTON_ID);

	protected final JoystickButton OperatorAButton = new JoystickButton(OperatorController, A_BUTTON_ID);
	protected final JoystickButton OperatorBButton = new JoystickButton(OperatorController, B_BUTTON_ID);
	protected final JoystickButton OperatorXButton = new JoystickButton(OperatorController, X_BUTTON_ID);
	protected final JoystickButton OperatorYButton = new JoystickButton(OperatorController, Y_BUTTON_ID);
	protected final JoystickButton OperatorLeftBumper = new JoystickButton(OperatorController, LEFT_BUMPER_ID);
	protected final JoystickButton OperatorRightBumper = new JoystickButton(OperatorController, RIGHT_BUMPER_ID);
	protected final JoystickButton OperatorLeftStickButton = new JoystickButton(OperatorController, LEFT_STICK_BUTTON_ID);
	protected final JoystickButton OperatorRightStickButton = new JoystickButton(OperatorController, RIGHT_STICK_BUTTON_ID);

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

        registerControls();

		scheduler.start();
	}

    public abstract void registerControls();

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
