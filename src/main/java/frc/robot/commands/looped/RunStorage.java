package frc.robot.commands.looped;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;

import static frc.robotmap.Constants.*;


public class RunStorage extends CommandBase {
	private final Storage storage;
	private final Alliance currentAlliance;
	private final ColorMatch colorMatcher = new ColorMatch();

	public RunStorage(Storage storage, Alliance currentAlliance) {
		this.storage = storage;
		this.currentAlliance = currentAlliance;
		colorMatcher.addColorMatch(BLUE_TARGET);
		colorMatcher.addColorMatch(RED_TARGET);
		addRequirements(storage);
	}

	@Override
	public void execute() {
		ColorMatchResult colorMatchResult = colorMatcher.matchClosestColor(storage.getColorSensor());

		if (currentAlliance == Alliance.Blue) {
			if (colorMatchResult.color.equals(BLUE_TARGET)) {
				storage.setCurrentColor(Storage.ColorSensorStatus.ALLIANCE);
			} else {
				storage.setCurrentColor(Storage.ColorSensorStatus.NOT_ALLIANCE);
			}
		} else {
			if (colorMatchResult.color.equals(RED_TARGET)) {
				storage.setCurrentColor(Storage.ColorSensorStatus.ALLIANCE);
			} else {
				storage.setCurrentColor(Storage.ColorSensorStatus.NOT_ALLIANCE);
			}
		}

		if (storage.getLineBreak()) {
			if (storage.getBallStatus() == Storage.BallStatus.INDEXING) {
				storage.setBallStatus(Storage.BallStatus.INDEXED);
			}
		} else {
			storage.setBallStatus(Storage.BallStatus.INDEXING);
		}

		if (!storage.getLineBreak()) {
			storage.runStorage(STORAGE_INDEX_SPEED);
		} else {
			storage.runStorage(0);
			if (storage.getBallStatus() == Storage.BallStatus.INDEXED) {
				if (storage.getCurrentColor() == Storage.ColorSensorStatus.NOT_ALLIANCE) {
					//TODO: Do outtake here
				}
			}
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
