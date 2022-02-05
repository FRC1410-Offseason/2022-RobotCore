package frc.robot.framework.scheduler.task;

import java.util.*;

import frc.robot.framework.scheduler.RobotMode;

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
