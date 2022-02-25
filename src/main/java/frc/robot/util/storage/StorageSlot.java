package frc.robot.util.storage;

import frc.robot.subsystems.Storage;

public class StorageSlot {

	/**
	 * Tracks whether there is currently a cargo in the slot
	 */
	private boolean ballPresent;

	/**
	 * Keeps track of the color of the cargo in the slot
	 * There is an EMPTY state that corresponds to neither red nor blue
	 */
	private Storage.ColorSensorStatus color;

	public StorageSlot() {
		ballPresent = false;
		color = Storage.ColorSensorStatus.EMPTY;
	}

	/**
	 * Sets if there is a cargo in the storage
	 * @param state true if there is a cargo, false if not
	 */
	public void setBallPresent(boolean state) {
		ballPresent = state;
	}

	/**
	 * Set the status of the slot
	 * @param status either ALLIANCE, NOT_ALLIANCE, and EMPTY
	 */
	public void setColor(Storage.ColorSensorStatus status) {
		color = status;
	}

	/**
	 * Returns if a ball is in the slot
	 * @return true if there is a ball and false otherwise
	 */
	public boolean getBallPresent() {
		return ballPresent;
	}

	public Storage.ColorSensorStatus getColor() {
		return color;
	}
}
