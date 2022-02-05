package frc.robot.framework.scheduler.task;

import frc.robot.framework.scheduler.RobotMode;

import java.util.EnumSet;
import java.util.Set;

@FunctionalInterface
public interface Task {

	void execute();

	default boolean isFinished() {
		return false;
	}

	default Set<RobotMode> getDisallowedModes() {
		return EnumSet.of(RobotMode.DISABLED);
	}
}
