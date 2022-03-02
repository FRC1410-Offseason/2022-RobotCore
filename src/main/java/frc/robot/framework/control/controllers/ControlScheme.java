package frc.robot.framework.control.controllers;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.framework.scheduler.TaskScheduler;
import edu.wpi.first.wpilibj.XboxController;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.*;

/**
 * A mixin used to receive button getters.
 */
public interface ControlScheme {

	XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
	XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);

	TaskScheduler getScheduler();

	void registerControls();

	// <editor-fold desc="> Driver buttons">
	default Button getDriverAButton() {
		return new Button(driverController, getScheduler(), ButtonID.A);
	}

	default Button getDriverBButton() {
		return new Button(driverController, getScheduler(), ButtonID.B);
	}

	default Button getDriverXButton() {
		return new Button(driverController, getScheduler(), ButtonID.X);
	}

	default Button getDriverYButton() {
		return new Button(driverController, getScheduler(), ButtonID.Y);
	}

	default Button getDriverLeftBumper() {
		return new Button(driverController, getScheduler(), ButtonID.LEFT_BUMPER);
	}

	default Button getDriverRightBumper() {
		return new Button(driverController, getScheduler(), ButtonID.RIGHT_BUMPER);
	}

	default Button getDriverLeftStickButton() {
		return new Button(driverController, getScheduler(), ButtonID.LEFT_STICK_BUTTON);
	}

	default Button getDriverRightStickButton() {
		return new Button(driverController, getScheduler(), ButtonID.RIGHT_STICK_BUTTON);
	}

    default Button getDriverDPadUp() {
		return new Button(driverController, getScheduler(), ButtonID.DPAD_UP);
	}

    default Button getDriverDPadDown() {
		return new Button(driverController, getScheduler(), ButtonID.DPAD_DOWN);
	}

    default Button getDriverDPadLeft() {
		return new Button(driverController, getScheduler(), ButtonID.DPAD_LEFT);
	}

    default Button getDriverDPadRight() {
		return new Button(driverController, getScheduler(), ButtonID.DPAD_RIGHT);
	}
	// </editor-fold>

	// <editor-fold desc="> Operator buttons">
	default Button getOperatorAButton() {
		return new Button(operatorController, getScheduler(), ButtonID.A);
	}

	default Button getOperatorBButton() {
		return new Button(operatorController, getScheduler(), ButtonID.B);
	}

	default Button getOperatorXButton() {
		return new Button(operatorController, getScheduler(), ButtonID.X);
	}

	default Button getOperatorYButton() {
		return new Button(operatorController, getScheduler(), ButtonID.Y);
	}

	default Button getOperatorLeftBumper() {
		return new Button(operatorController, getScheduler(), ButtonID.LEFT_BUMPER);
	}

	default Button getOperatorRightBumper() {
		return new Button(operatorController, getScheduler(), ButtonID.RIGHT_BUMPER);
	}

	default Button getOperatorLeftStickButton() {
		return new Button(operatorController, getScheduler(), ButtonID.LEFT_STICK_BUTTON);
	}

	default Button getOperatorRightStickButton() {
		return new Button(operatorController, getScheduler(), ButtonID.RIGHT_STICK_BUTTON);
	}

	default Button getOperatorDPadUp() {
		return new Button(operatorController, getScheduler(), ButtonID.DPAD_UP);
	}

    default Button getOperatorDPadDown() {
		return new Button(operatorController, getScheduler(), ButtonID.DPAD_DOWN);
	}

    default Button getOperatorDPadLeft() {
		return new Button(operatorController, getScheduler(), ButtonID.DPAD_LEFT);
	}

    default Button getOperatorDPadRight() {
		return new Button(operatorController, getScheduler(), ButtonID.DPAD_RIGHT);
	}
	// </editor-fold>

	// <editor-fold desc="> Driver axes">
	default Axis getDriverLeftXAxis() {
		return new Axis(driverController, AxisID.LEFT_X, DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverRightXAxis() {
		return new Axis(driverController, AxisID.RIGHT_X, DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverLeftYAxis() {
		return new Axis(driverController, AxisID.LEFT_Y, DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverRightYAxis() {
		return new Axis(driverController, AxisID.RIGHT_Y, DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverLeftTrigger() {
		return new Axis(driverController, AxisID.LEFT_TRIGGER, DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverRightTrigger() {
		return new Axis(driverController, AxisID.RIGHT_TRIGGER, DRIVER_DEADZONE_VALUE);
	}
	// </editor-fold>

	// <editor-fold desc="> Operator axes">
	default Axis getOperatorLeftXAxis() {
		return new Axis(operatorController, AxisID.LEFT_X, OPERATOR_DEADZONE_VALUE);
	}

	default Axis getOperatorRightXAxis() {
		return new Axis(operatorController, AxisID.RIGHT_X, OPERATOR_DEADZONE_VALUE);
	}

	default Axis getOperatorLeftYAxis() {
		return new Axis(operatorController, AxisID.LEFT_Y, OPERATOR_DEADZONE_VALUE);
	}

	default Axis getOperatorRightYAxis() {
		return new Axis(operatorController, AxisID.RIGHT_Y, OPERATOR_DEADZONE_VALUE);
	}

	default Axis getOperatorLeftTrigger() {
		return new Axis(operatorController, AxisID.LEFT_TRIGGER, OPERATOR_DEADZONE_VALUE);
	}

	default Axis getOperatorRightTrigger() {
		return new Axis(operatorController, AxisID.RIGHT_TRIGGER, OPERATOR_DEADZONE_VALUE);
	}
	// </editor-fold>
}
