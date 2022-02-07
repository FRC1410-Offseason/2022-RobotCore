package frc.robot.framework.scheduler.task.button;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.framework.control.ButtonStateObserver;
import frc.robot.framework.control.ButtonStateObserver.ButtonState;
import frc.robot.framework.scheduler.RobotMode;
import frc.robot.framework.scheduler.task.Task;

public class ToggleWhenPressedTask implements Task {

    private final ButtonStateObserver observer;
    private final Command command;

    private boolean running = false;

    public ToggleWhenPressedTask(ButtonStateObserver observer, Command command) {
        this.observer = observer;
        this.command = command;
    }

    @Override
	public void execute() {
        observer.updateState();

        if (observer.getState() == ButtonState.PRESSED) {
            if (!running) {
                command.initialize();
                running = true;
            } else {
                command.end(true);
                running = false;
            }
        }

		if (running) command.execute();

        if (running && command.isFinished()){
            command.end(false);
            running = false;
        }
	}

    @Override
	public Set<RobotMode> getDisallowedModes() {
		return Set.of(RobotMode.DISABLED, RobotMode.AUTONOMOUS);
	}
}
