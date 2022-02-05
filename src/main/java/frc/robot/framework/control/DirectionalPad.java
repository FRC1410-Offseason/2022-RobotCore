package frc.robot.framework.control;

import edu.wpi.first.wpilibj.XboxController;

public class DirectionalPad {
	private final XboxController controller;

	public DirectionalPad(XboxController controller) {
		this.controller = controller;
	}

	public int getRawPOV() {
		return controller.getPOV();
	}

	public Direction getDirection() {
		switch (getRawPOV()) {
			case 0: return Direction.UP;
			case 90: return Direction.RIGHT;
			case 180: return Direction.DOWN;
			case 270: return Direction.LEFT;

			case -1: default: return Direction.UNKNOWN;
		}
	}

	public enum Direction {
		UNKNOWN(-1),
		UP(0),
		RIGHT(90),
		DOWN(180),
		LEFT(270);

		final int pov;

		Direction(int pov) {
			this.pov = pov;
		}
	}
}
