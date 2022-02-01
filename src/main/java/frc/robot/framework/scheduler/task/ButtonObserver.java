package frc.robot.framework.scheduler.task;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.framework.scheduler.*;

import java.util.Set;

public class ButtonObserver implements Task {
	private final XboxController controller;
	private final XboxController.Button button;
	private final Command command;
	private boolean wasActive = false;

	public ButtonObserver(XboxController controller, XboxController.Button button, Command command) {
		this.controller = controller;
		this.button = button;
		this.command = command;
	}

	@Override
	public void execute() {
		if (controller.getRawButton(button.value)) {
			if (!wasActive) {
				wasActive = true;
				command.initialize();
			} else {
				command.execute();
			}
		} else if (wasActive) {
			command.end(true);
			wasActive = false;
		}
	}

	@Override
	public Set<RobotMode> getDisallowedModes() {
		return Set.of(RobotMode.DISABLED, RobotMode.AUTONOMOUS);
	}
}
