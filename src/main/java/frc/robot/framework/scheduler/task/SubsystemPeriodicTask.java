package frc.robot.framework.scheduler.task;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.framework.scheduler.RobotMode;
import frc.robot.framework.scheduler.SubsystemRegistry;

import java.util.Set;

public class SubsystemPeriodicTask implements Task {

	@Override
	public void execute() {
		SubsystemRegistry.getEntries().forEach(Subsystem::periodic);

		if (RobotBase.isSimulation()) {
			SubsystemRegistry.getEntries().forEach(Subsystem::simulationPeriodic);
		}
	}

	@Override
	public Set<RobotMode> getDisallowedModes() {
		return Set.of(RobotMode.DISABLED);
	}
}
