package frc.robot.framework.control;

import edu.wpi.first.wpilibj.XboxController;

import static frc.robotmap.IDs.*;

public class Axis {

    private final XboxController controller;
    private final AXIS_ID id;
    private final double deadzone;

    public Axis(XboxController controller, AXIS_ID id, double deadzone) {
        this.controller = controller;        
        this.id = id;
        this.deadzone = deadzone;
    }
    
    public double getRaw() {
        double result;

        switch(id) {
            case LEFT_X: result = controller.getLeftX(); break;
            case RIGHT_X: result = controller.getRightX(); break;
            case LEFT_Y: result = -controller.getLeftY(); break;
            case RIGHT_Y: result = -controller.getRightY(); break;               
            case LEFT_TRIGGER: result = controller.getLeftTriggerAxis(); break;
            case RIGHT_TRIGGER: result = controller.getRightTriggerAxis(); break;

            default: result = 0;
        }

        return result;
    }

    public double getOpposite() {
        double result;
        
        switch (id) {
            case LEFT_X: result = -controller.getLeftY(); break;
            case RIGHT_X: result = -controller.getRightY(); break;
            case LEFT_Y: result = controller.getLeftX(); break;
            case RIGHT_Y: result = controller.getRightX(); break;            

            default: result = 0;
        }

        return result;
    }

    public double getDeadzoned() {
        double value = getRaw();
		double oppositeValue = getOpposite();

		double magnitude = Math.sqrt(Math.pow(value, 2) + Math.pow(oppositeValue, 2));

		if (magnitude <= deadzone) {
			return 0;
		} else {
			return -((magnitude - deadzone) / (1 - deadzone)) * (value / magnitude);
		}
	}
}

