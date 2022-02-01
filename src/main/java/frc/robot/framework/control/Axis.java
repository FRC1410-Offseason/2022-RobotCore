package frc.robot.framework.control.input;

import edu.wpi.first.wpilibj.XboxController;
import static frc.robotmap.IDs.*;

public class Axis {

    private final XboxController controller;
    private final AxisId id;
    private final double deadzone;

    public Axis(XboxController controller, AxisId id, double deadzone) {
        
        this.controller = controller;        
        this.id = id;
        this.deadzone = deadzone;
    }
    
    public double getRaw() {
        double result;

        switch(id) {
            case LEFT_X: result = controller.getLeftX();
            case RIGHT_X: result = controller.getRightX();
            case LEFT_Y: result = controller.getLeftY();
            case RIGHT_Y: result = controller.getRightY();                
            case LEFT_TRIGGER: result = controller.getLeftTriggerAxis();
            case RIGHT_TRIGGER: result = controller.getRightTriggerAxis();

            default: result = 0;

        }

        return result;
    }

    public double getOpposite() {
        double result;
        
        switch (id) {
            case LEFT_X: result = -controller.getLeftY();
            case RIGHT_X: result = -controller.getRightY();
            case LEFT_Y: result = controller.getLeftX();
            case RIGHT_Y: result = controller.getRightX();                

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

