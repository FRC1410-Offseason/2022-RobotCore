package frc.robot.framework.scheduler.task;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.framework.scheduler.*;

public class SubsystemPeriodicTask implements Task {

	@Override
	public void execute() {
		SubsystemRegistry.getEntries().forEach(Subsystem::periodic);
	}
}
