package frc.robot.framework.control;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.framework.scheduler.TaskScheduler;
import frc.robotmap.*;

/**
 * A mixin used to receive button getters.
 */
public interface ControlScheme {
	XboxController driverController = new XboxController(IDs.DRIVER_CONTROLLER_PORT);
	XboxController operatorController = new XboxController(IDs.OPERATOR_CONTROLLER_PORT);

	TaskScheduler getScheduler();

	void registerControls();

	// <editor-fold desc="> Driver buttons">
	default Button getDriverAButton() {
		return new Button(driverController, getScheduler(), IDs.ButtonId.A);
	}

	default Button getDriverBButton() {
		return new Button(driverController, getScheduler(), IDs.ButtonId.B);
	}

	default Button getDriverXButton() {
		return new Button(driverController, getScheduler(), IDs.ButtonId.X);
	}

	default Button getDriverYButton() {
		return new Button(driverController, getScheduler(), IDs.ButtonId.Y);
	}

	default Button getDriverLeftBumper() {
		return new Button(driverController, getScheduler(), IDs.ButtonId.LEFT_BUMPER);
	}

	default Button getDriverRightBumper() {
		return new Button(driverController, getScheduler(), IDs.ButtonId.RIGHT_BUMPER);
	}

	default Button getDriverLeftStickButton() {
		return new Button(driverController, getScheduler(), IDs.ButtonId.LEFT_STICK_BUTTON);
	}

	default Button getDriverRightStickButton() {
		return new Button(driverController, getScheduler(), IDs.ButtonId.RIGHT_STICK_BUTTON);
	}
	// </editor-fold>

	// <editor-fold desc="> Operator buttons">
	default Button getOperatorAButton() {
		return new Button(operatorController, getScheduler(), IDs.ButtonId.A);
	}

	default Button getOperatorBButton() {
		return new Button(operatorController, getScheduler(), IDs.ButtonId.B);
	}

	default Button getOperatorXButton() {
		return new Button(operatorController, getScheduler(), IDs.ButtonId.X);
	}

	default Button getOperatorYButton() {
		return new Button(operatorController, getScheduler(), IDs.ButtonId.Y);
	}

	default Button getOperatorLeftBumper() {
		return new Button(operatorController, getScheduler(), IDs.ButtonId.LEFT_BUMPER);
	}

	default Button getOperatorRightBumper() {
		return new Button(operatorController, getScheduler(), IDs.ButtonId.RIGHT_BUMPER);
	}

	default Button getOperatorLeftStickButton() {
		return new Button(operatorController, getScheduler(), IDs.ButtonId.LEFT_STICK_BUTTON);
	}
    
	default Button getOperatorRightStickButton() {
		return new Button(operatorController, getScheduler(), IDs.ButtonId.RIGHT_STICK_BUTTON);
	}
	// </editor-fold>

	// <editor-fold desc="> Driver axes">
	default Axis getDriverLeftXAxis() {
		return new Axis(driverController, IDs.AxisId.LEFT_X, Tuning.DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverRightXAxis() {
		return new Axis(driverController, IDs.AxisId.RIGHT_X, Tuning.DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverLeftYAxis() {
		return new Axis(driverController, IDs.AxisId.LEFT_Y, Tuning.DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverRightYAxis() {
		return new Axis(driverController, IDs.AxisId.RIGHT_Y, Tuning.DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverLeftTrigger() {
		return new Axis(driverController, IDs.AxisId.LEFT_TRIGGER, Tuning.DRIVER_DEADZONE_VALUE);
	}

	default Axis getDriverRightTrigger() {
		return new Axis(driverController, IDs.AxisId.RIGHT_TRIGGER, Tuning.DRIVER_DEADZONE_VALUE);
	}
	// </editor-fold>

	// <editor-fold desc="> Operator axes">
	default Axis getOperatorLeftXAxis() {
		return new Axis(operatorController, IDs.AxisId.LEFT_X, Tuning.DRIVER_DEADZONE_VALUE);
	}

	default Axis getOperatorRightXAxis() {
		return new Axis(operatorController, IDs.AxisId.RIGHT_X, Tuning.DRIVER_DEADZONE_VALUE);
	}

	default Axis getOperatorLeftYAxis() {
		return new Axis(operatorController, IDs.AxisId.LEFT_Y, Tuning.DRIVER_DEADZONE_VALUE);
	}

	default Axis getOperatorRightYAxis() {
		return new Axis(operatorController, IDs.AxisId.RIGHT_Y, Tuning.DRIVER_DEADZONE_VALUE);
	}

	default Axis getOperatorLeftTrigger() {
		return new Axis(operatorController, IDs.AxisId.LEFT_TRIGGER, Tuning.DRIVER_DEADZONE_VALUE);
	}

	default Axis getOperatorRightTrigger() {
		return new Axis(operatorController, IDs.AxisId.RIGHT_TRIGGER, Tuning.DRIVER_DEADZONE_VALUE);
	}
	// </editor-fold>
}
