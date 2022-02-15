package frc.robot.framework.scheduler.task.button;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.framework.control.ButtonStateObserver;
import frc.robot.framework.control.ButtonStateObserver.ButtonState;
import frc.robot.framework.scheduler.RobotMode;
import frc.robot.framework.scheduler.task.Task;

import java.util.Set;

public class WhileHeldTask implements Task {
    
    private final ButtonStateObserver observer;
    private final Command command;

    private boolean running = false;

    public WhileHeldTask(ButtonStateObserver observer, Command command) {
        this.observer = observer;
        this.command = command;
    }

	@Override
	public void execute() {
        observer.updateState();

		if (!running && observer.getRaw()) {
            command.initialize();
            running = true;
        } else if (running && observer.getState() == ButtonState.RELEASED) {
            command.end(true);
            running = false;
        }

		if (running) {
			command.execute();
			if (command.isFinished()) {
				running = false;
				command.end(false);
			}
		}
	}

    @Override
	public Set<RobotMode> getDisallowedModes() {
		return Set.of(RobotMode.DISABLED, RobotMode.AUTONOMOUS);
	}
}
