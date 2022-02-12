package frc.robot.util.storage;

import frc.robot.subsystems.Storage.ColorSensorStatus;

public class StorageState {

	/**
	 * We have the capability to store 2 cargo in the storage, so we need two slot objects
	 */
	private StorageSlot slot1;
	private StorageSlot slot2;

	public StorageState() {
		slot1 = new StorageSlot();
		slot2 = new StorageSlot();
	}

	public int getNumCargo() {
		int num = 0;
		num += slot1.getBallPresent() ? 1 : 0;
		num += slot2.getBallPresent() ? 1 : 0;
		return num;
	}

	/**
	 * Set the alliance of the first slot
	 * @param alliance True -> Correct Alliance, False -> Incorrect Alliance
	 */
	public void setSlot1(boolean alliance) {
		slot1.setBallPresent(true);
		slot1.setColor(alliance ? ColorSensorStatus.ALLIANCE : ColorSensorStatus.NOT_ALLIANCE);
	}

	/**
	 * Reset the first slot, used for outtaking and the updating of the internal storage state
	 */
	public void resetSlot1() {
		slot1.setBallPresent(false);
		slot1.setColor(ColorSensorStatus.EMPTY);
	}

	/**
	 * Reset the second slot, used for outtaking and the updating of the internal storage state
	 */
	public void resetSlot2() {
		slot1.setBallPresent(false);
		slot1.setColor(ColorSensorStatus.EMPTY);
	}

	/**
	 * Set the alliance of the first slot
	 * @param alliance True -> Correct Alliance, False ->  Incorrect Alliance
	 */
	public void setSlot2(boolean alliance) {
		slot2.setBallPresent(true);
		slot2.setColor(alliance ? ColorSensorStatus.ALLIANCE : ColorSensorStatus.NOT_ALLIANCE);
	}

	/**
	 * Get the first storage slot
	 * @return a StorageSlot that corresponds to the part of the storage closest to the intake
	 */
	public StorageSlot getSlot1() {
		return slot1;
	}

	/**
	 * Get the second storage slot
	 * @return a StorageSlot that corresponds to the part of the storage closest to the shooter
	 */
	public StorageSlot getSlot2() {
		return slot2;
	}
}
