package frc.robot.framework.control;

import edu.wpi.first.wpilibj.XboxController;

public class D {
	private final XboxController controller;

	public D(XboxController controller) {
		this.controller = controller;
	}

	public int getAngle() {
		return controller.getPOV();
	}
}
