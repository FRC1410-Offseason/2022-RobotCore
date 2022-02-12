package frc.robot.framework.scheduler;

import java.util.EnumSet;
import java.util.Set;

public interface Observer {

    final EnqueuedTask task = null;

    void check();

    default Set<RobotMode> getDisallowedModes() {
		return EnumSet.of(RobotMode.DISABLED);
	}
}
