package frc.robot.framework.control;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.framework.scheduler.TaskScheduler;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.DRIVER_DEADZONE_VALUE;

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
		return new Button(driverController, getScheduler(), ButtonId.A);
	}

	default Button getDriverBButton() {
		return new Button(driverController, getScheduler(), ButtonId.B);
	}

	default Button getDriverXButton() {
		return new Button(driverController, getScheduler(), ButtonId.X);
	}

	default Button getDriverYButton() {
		return new Button(driverController, getScheduler(), ButtonId.Y);
	}

	default Button getDriverLeftBumper() {
		return new Button(driverController, getScheduler(), ButtonId.LEFT_BUMPER);
	}

	default Button getDriverRightBumper() {
		return new Button(driverController, getScheduler(), ButtonId.RIGHT_BUMPER);
	}

	default Button getDriverLeftStickButton() {
		return new Button(driverController, getScheduler(), ButtonId.LEFT_STICK_BUTTON);
	}

	default Button getDriverRightStickButton() {
		return new Button(driverController, getScheduler(), ButtonId.RIGHT_STICK_BUTTON);
	}
	// </editor-fold>

	// <editor-fold desc="> Operator buttons">
	default Button getOperatorAButton() {
		return new Button(operatorController, getScheduler(), ButtonId.A);
	}

	default Button getOperatorBButton() {
		return new Button(operatorController, getScheduler(), ButtonId.B);
	}

	default Button getOperatorXButton() {
		return new Button(operatorController, getScheduler(), ButtonId.X);
	}

	default Button getOperatorYButton() {
		return new Button(operatorController, getScheduler(), ButtonId.Y);
	}

	default Button getOperatorLeftBumper() {
		return new Button(operatorController, getScheduler(), ButtonId.LEFT_BUMPER);
	}

	default Button getOperatorRightBumper() {
		return new Button(operatorController, getScheduler(), ButtonId.RIGHT_BUMPER);
	}

	default Button getOperatorLeftStickButton() {
		return new Button(operatorController, getScheduler(), ButtonId.LEFT_STICK_BUTTON);
	}

	default Button getOperatorRightStickButton() {
		return new Button(operatorController, getScheduler(), ButtonId.RIGHT_STICK_BUTTON);
	}
	// </editor-fold>

	// <editor-fold desc="> Driver axes">
	default Axis getDriverLeftXAxis() {
		return new Axis(driverController, AXIS_ID.LEFT_X, DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverRightXAxis() {
		return new Axis(driverController, AXIS_ID.RIGHT_X, DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverLeftYAxis() {
		return new Axis(driverController, AXIS_ID.LEFT_Y, DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverRightYAxis() {
		return new Axis(driverController, AXIS_ID.RIGHT_Y, DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverLeftTrigger() {
		return new Axis(driverController, AXIS_ID.LEFT_TRIGGER, DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverRightTrigger() {
		return new Axis(driverController, AXIS_ID.RIGHT_TRIGGER, DRIVER_DEADZONE_VALUE);
	}
	// </editor-fold>

	// <editor-fold desc="> Operator axes">
	default Axis getOperatorLeftXAxis() {
		return new Axis(operatorController, AXIS_ID.LEFT_X, DRIVER_DEADZONE_VALUE);
	}

	default Axis getOperatorRightXAxis() {
		return new Axis(operatorController, AXIS_ID.RIGHT_X, DRIVER_DEADZONE_VALUE);
	}

	default Axis getOperatorLeftYAxis() {
		return new Axis(operatorController, AXIS_ID.LEFT_Y, DRIVER_DEADZONE_VALUE);
	}

	default Axis getOperatorRightYAxis() {
		return new Axis(operatorController, AXIS_ID.RIGHT_Y, DRIVER_DEADZONE_VALUE);
	}

	default Axis getOperatorLeftTrigger() {
		return new Axis(operatorController, AXIS_ID.LEFT_TRIGGER, DRIVER_DEADZONE_VALUE);
	}

	default Axis getOperatorRightTrigger() {
		return new Axis(operatorController, AXIS_ID.RIGHT_TRIGGER, DRIVER_DEADZONE_VALUE);
	}
	// </editor-fold>
}
