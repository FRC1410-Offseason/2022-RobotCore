package frc.robot.framework.scheduler.task.button;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.framework.scheduler.task.Task;

public abstract class ButtonTask implements Task {
	final Command command;
	final XboxController controller;
	final int id;

	public ButtonTask(Command command, XboxController controller, int id) {
		this.command = command;
		this.controller = controller;
		this.id = id;
	}

	public boolean isActive() {
		return controller.getRawButton(id);
	}
}
