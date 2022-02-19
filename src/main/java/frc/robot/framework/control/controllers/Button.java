package frc.robot.framework.control.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.framework.control.observers.ButtonStateObserver;
import frc.robot.framework.control.observers.ButtonStateObserver.ButtonStateCondition;
import frc.robot.framework.scheduler.TaskScheduler;
import static frc.robotmap.IDs.*;

public class Button {

	private final XboxController controller;
	private final TaskScheduler scheduler;
	private final BUTTON_ID id;

	public Button(XboxController controller, TaskScheduler scheduler, BUTTON_ID button) {
		this.controller = controller;
		this.scheduler = scheduler;
		this.id = button;
	}

	public void whenPressed(Command command) {
		scheduler.queueObservedCommand(command, new ButtonStateObserver(ButtonStateCondition.WHEN_PRESSED, controller, id));
	}

    public void whenReleased(Command command) {
        scheduler.queueObservedCommand(command, new ButtonStateObserver(ButtonStateCondition.WHEN_RELEASED, controller, id));
    }

    public void toggleWhenPressed(Command command) {
        scheduler.queueObservedCommand(command, new ButtonStateObserver(ButtonStateCondition.TOGGLE_WHEN_PRESSED, controller, id));
    }

    public void whileHeld(Command command) {
        scheduler.queueObservedCommand(command, new ButtonStateObserver(ButtonStateCondition.WHILE_HELD, controller, id));
    }
}
