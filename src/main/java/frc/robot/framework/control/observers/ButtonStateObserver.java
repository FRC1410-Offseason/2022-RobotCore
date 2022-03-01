package frc.robot.framework.control.observers;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.framework.scheduler.task.Task;

import static frc.robotmap.IDs.*;

public class ButtonStateObserver extends Observer {

    private final XboxController controller;
	private final BUTTON_ID id;

    private boolean pressed = false;
    private boolean wasPressed = false;

    private boolean running = false;

    private ButtonState buttonState = ButtonState.UNCHANGED;
    private ButtonStateCondition stateToCheck = null;

	public ButtonStateObserver(ButtonStateCondition stateToCheck, XboxController controller, BUTTON_ID id) {
		this.stateToCheck = stateToCheck;
        this.controller = controller;
		this.id = id;

        configurePriority(SCHEDULER_PRIORITY.HIGH);
	}

    @Override
    public void check() {
        updateButtonPressedState();

        switch (stateToCheck) {
            case WHEN_PRESSED: 
                if (buttonState == ButtonState.PRESSED) requestExecution(); 
                break;
            case WHEN_RELEASED:
                if (buttonState == ButtonState.RELEASED) requestExecution();
                break;
            case WHILE_HELD:
                if (buttonState == ButtonState.PRESSED) requestExecution(); 
                if (buttonState == ButtonState.RELEASED) requestCancellation();
                break;
            case TOGGLE_WHEN_PRESSED:
                //Cancel toggle cycle if all tasks are disabled
                boolean isEnabled = false;
                for (Task task : boundTaskList) {
                    if (task.isEnabled() || task.isRequestingExecution()) isEnabled = true;
                }
                if (!isEnabled) running = false;
            
                if (buttonState == ButtonState.PRESSED) {
                    if (!running) {
                        requestExecution();
                        running = true;
                    } else {
                        requestCancellation();
                        running = false;
                    }
                }
                break;

            default: break;
        }
    }

    public boolean getRaw() {
        switch (id) {
            case A: return controller.getAButton();
            case B: return controller.getBButton();
            case X: return controller.getXButton();
            case Y: return controller.getYButton();
            case LEFT_BUMPER: return controller.getLeftBumper();
            case RIGHT_BUMPER: return controller.getRightBumper();
            case LEFT_STICK_BUTTON: return controller.getLeftStickButton();
            case RIGHT_STICK_BUTTON: return controller.getRightStickButton();
            case DPAD_UP: return controller.getPOV() == 0;
            case DPAD_DOWN: return controller.getPOV() == 180;
            case DPAD_LEFT: return controller.getPOV() == 270;
            case DPAD_RIGHT: return controller.getPOV() == 90;

            default: return false;
        }
    }

    public void updateButtonPressedState() {
        pressed = getRaw();

        if (pressed && !wasPressed) {
            buttonState = ButtonState.PRESSED;
        } else if (!pressed && wasPressed) {
            buttonState = ButtonState.RELEASED;
        } else {
            buttonState = ButtonState.UNCHANGED;
        }

        wasPressed = pressed;
    }

    public enum ButtonState {
        PRESSED,
        RELEASED,
        UNCHANGED
    }

    public enum ButtonStateCondition {
        WHEN_PRESSED,
        WHEN_RELEASED,
        WHILE_HELD,
        TOGGLE_WHEN_PRESSED
    }
}
