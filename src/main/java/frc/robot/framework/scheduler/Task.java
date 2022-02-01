package frc.robot.framework.scheduler;

import java.util.*;

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
