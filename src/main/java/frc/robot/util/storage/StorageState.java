package frc.robot.util.storage;

import frc.robot.subsystems.Storage.ColorSensorStatus;

public class StorageState {

	//We have the capacity for 2 cargo, so there are two slots in the storage
	private StorageSlot slot1;
	private StorageSlot slot2;

	public StorageState() {
		slot1 = new StorageSlot();
		slot2 = new StorageSlot();
	}

	public void setSlot1(boolean alliance) {
		slot1.setBallPresent(true);
		slot1.setColor(alliance ? ColorSensorStatus.ALLIANCE : ColorSensorStatus.NOT_ALLIANCE);
	}

	public void resetSlot1() {
		slot1.setBallPresent(false);
		slot1.setColor(ColorSensorStatus.EMPTY);
	}

	public void resetSlot2() {
		slot1.setBallPresent(false);
		slot1.setColor(ColorSensorStatus.EMPTY);
	}

	public void setSlot2(boolean alliance) {
		slot2.setBallPresent(true);
		slot2.setColor(alliance ? ColorSensorStatus.ALLIANCE : ColorSensorStatus.NOT_ALLIANCE);
	}

	public StorageSlot getSlot1() {
		return slot1;
	}

	public StorageSlot getSlot2() {
		return slot2;
	}
}
