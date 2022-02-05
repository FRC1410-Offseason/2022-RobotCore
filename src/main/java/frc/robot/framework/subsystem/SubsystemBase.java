package frc.robot.framework.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.framework.scheduler.SubsystemRegistry;

public class SubsystemBase implements Subsystem {

	public SubsystemBase() {
		register();
	}

	@Override
	public void register() {
		SubsystemRegistry.register(this);
	}
}
