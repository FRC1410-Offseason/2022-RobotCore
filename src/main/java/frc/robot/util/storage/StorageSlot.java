package frc.robot.util.storage;

import frc.robot.subsystems.Storage;

public class StorageSlot {
	private boolean ballPresent;
	private Storage.ColorSensorStatus color;

	public StorageSlot() {
		ballPresent = false;
		color = Storage.ColorSensorStatus.EMPTY;
	}

	public void setBallPresent(boolean state) {
		ballPresent = state;
	}

	public void setColor(Storage.ColorSensorStatus status) {
		color = status;
	}

	public boolean getBallPresent() {
		return ballPresent;
	}

	public Storage.ColorSensorStatus getColor() {
		return color;
	}
}
