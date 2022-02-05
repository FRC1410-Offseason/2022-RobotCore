package frc.robot.framework.control;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.framework.scheduler.TaskScheduler;
import frc.robot.framework.scheduler.task.button.*;
import static frc.robotmap.IDs.*;

public class Button {

	private final XboxController controller;
	private final TaskScheduler scheduler;
	private final ButtonId id;

	public Button(XboxController controller, TaskScheduler scheduler, ButtonId button) {
		this.controller = controller;
		this.scheduler = scheduler;
		this.id = button;
	}

	public void whenPressed(Command command) {
		scheduler.queuePeriodic(new WhenPressedTask(new ButtonStateObserver(controller, id), command));
	}

    public void whenUnpressed(Command command) {
		scheduler.queuePeriodic(new WhenUnpressedTask(new ButtonStateObserver(controller, id), command));
	}

    public void toggleWhenPressed(Command command) {
		scheduler.queuePeriodic(new ToggleWhenPressedTask(new ButtonStateObserver(controller, id), command));
	}

    public void whileHeld(Command command) {
		scheduler.queuePeriodic(new WhileHeldTask(new ButtonStateObserver(controller, id), command));
	}
}
