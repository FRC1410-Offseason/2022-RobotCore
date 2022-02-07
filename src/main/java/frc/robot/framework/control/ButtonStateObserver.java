package frc.robot.framework.control;

import edu.wpi.first.wpilibj.XboxController;

import static frc.robotmap.IDs.*;

public class ButtonStateObserver {

    private final XboxController controller;
	private final ButtonId id;

    private boolean pressed = false;
    private boolean wasPressed = false;

    private ButtonState internalButtonState = ButtonState.UNCHANGED;

	public ButtonStateObserver(XboxController controller, ButtonId id) {
		this.controller = controller;
		this.id = id;
	}

    public enum ButtonState {
        PRESSED,
        RELEASED,
        UNCHANGED
    }

    public void updateState() {
        pressed = getRaw();

        if (pressed && !wasPressed) {
            internalButtonState = ButtonState.PRESSED;
        } else if (!pressed && wasPressed) {
            internalButtonState = ButtonState.RELEASED;
        } else {
            internalButtonState = ButtonState.UNCHANGED;
        }

        wasPressed = pressed;
    }

    public ButtonState getState() {
        return internalButtonState;
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
}
