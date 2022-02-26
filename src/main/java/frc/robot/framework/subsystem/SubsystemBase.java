package frc.robot.framework.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class SubsystemBase implements Subsystem {

	public SubsystemBase() {
		register();
	}

	@Override
	public void register() {
		SubsystemRegistry.register(this);
	}
}
