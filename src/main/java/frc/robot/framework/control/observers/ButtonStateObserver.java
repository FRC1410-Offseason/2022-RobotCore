package frc.robot.framework.control.observers;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.framework.scheduler.EnqueuedTask;
import frc.robot.framework.scheduler.Observer;

import static frc.robotmap.IDs.*;

public class ButtonStateObserver extends Observer {

    private final XboxController controller;
	private final ButtonId id;

    private boolean pressed = false;
    private boolean wasPressed = false;

    private boolean running = false;

    private ButtonState buttonState = ButtonState.UNCHANGED;
    private StateCondition stateToCheck = null;

	public ButtonStateObserver(XboxController controller, ButtonId id) {
		this.controller = controller;
		this.id = id;
	}

    public ButtonStateObserver(XboxController controller, ButtonId id, EnqueuedTask task) {
		this.controller = controller;
		this.id = id;

        this.bind(task);
	}

    public void bindTask(EnqueuedTask task) {
        bind(task);
    }

    public void check() {
        updateState();

        switch (stateToCheck) {
            case WHEN_PRESSED: 
                if (buttonState == ButtonState.PRESSED) task.enable(); 
                break;
            case WHEN_RELEASED:
                if (buttonState == ButtonState.RELEASED) task.enable();
                break;
            case WHILE_HELD:
                if (buttonState == ButtonState.PRESSED) task.enable(); 
                if (buttonState == ButtonState.RELEASED) task.disable();
                break;
            case TOGGLE_WHEN_PRESSED:
                if (buttonState == ButtonState.PRESSED) {
                    if (!running) {
                        task.enable();
                        running = true;
                    } else {
                        task.disable();
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

    public void updateState() {
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

    private enum StateCondition {
        WHEN_PRESSED,
        WHEN_RELEASED,
        WHILE_HELD,
        TOGGLE_WHEN_PRESSED
    }
}
