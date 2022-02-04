package frc.robot.framework.control;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.framework.scheduler.TaskScheduler;
import frc.robot.framework.scheduler.task.ButtonObserver;

import frc.robotmap.IDs;

public class Button {
	private final XboxController controller;
	private final TaskScheduler scheduler;
	private final int id;

	public Button(XboxController controller, TaskScheduler scheduler, IDs.ButtonId button) {
		this.controller = controller;
		this.scheduler = scheduler;
		this.id = button.getId();
	}

	public void whileHeld(Command command) {
		scheduler.queuePeriodic(new ButtonObserver(controller, id, command));
	}
}
