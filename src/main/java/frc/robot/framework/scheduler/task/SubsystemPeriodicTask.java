package frc.robot.framework.scheduler.task;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.framework.subsystem.SubsystemRegistry;

public class SubsystemPeriodicTask extends Task {

	@Override
	public void execute() {
		SubsystemRegistry.getEntries().forEach(Subsystem::periodic);

		if (RobotBase.isSimulation()) {
			SubsystemRegistry.getEntries().forEach(Subsystem::simulationPeriodic);
		}
	}
}
