package frc.robot.framework.scheduler.task.button;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class SingleButtonTask extends ButtonTask {

	private boolean wasActive = false;

	public SingleButtonTask(Command command, XboxController controller, int id) {
		super(command, controller, id);
	}

	@Override
	public void execute() {
		if (isActive()) {
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
}
