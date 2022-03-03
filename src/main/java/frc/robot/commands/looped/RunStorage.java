package frc.robot.commands.looped;

import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;

import static frc.robotmap.Constants.*;

public class RunStorage extends CommandBase {

	private final Storage storage;

	public RunStorage(Storage storage) {
		this.storage = storage;
		addRequirements(storage);
	}

	@Override
	public void execute() {
		if (!storage.isManualControl()) {
			// Falling edge
			if (storage.isLineBreakPrev() && !storage.getLineBreak()) {
				storage.runStorage(STORAGE_RUN_SPEED);
				// If there is a ball in the first position, then move it to the second
				// and reset the first position to be ready for the next ball
				if (storage.getCurrentState().getSlot1().getBallPresent()) {
					storage.getCurrentState().setSlot2(true);
					storage.getCurrentState().resetSlot1();
				}
				// Rising Edge
			} else if (!storage.isLineBreakPrev() && storage.getLineBreak()) {
				// Stop motor so we can read the color
				storage.runStorage(0);
				// Read the color from the color sensor
				ColorMatchResult result = storage.getColorMatch().matchClosestColor(storage.getColor());
				if (result.color.equals(RED_TARGET)) {
					if (storage.getCurrentAlliance() == DriverStation.Alliance.Red) {
						// If the detected color is red, and we're on the red alliance, then the color is correct
						storage.getCurrentState().setSlot1(true);
					} else {
						// If the detected color is red, and we're on the blue alliance, then the color is incorrect
						storage.getCurrentState().setSlot1(false);
						// If the color is wrong then we need to set the outtake flag for the outtake handler
						storage.setOuttakeFlag(true);
					}
				} else {
					if (storage.getCurrentAlliance() == DriverStation.Alliance.Red) {
						// If the detected color is blue, and we're on the red alliance, then the color is incorrect
						storage.getCurrentState().setSlot1(false);
						// If the color is wrong then we need to set the outtake flag for the outtake handler
						storage.setOuttakeFlag(true);
					} else {
						// If the detected color is blue, and we're on the blue alliance, then the color is correct
						storage.getCurrentState().setSlot1(true);
					}
				}
			} else {
				if (storage.isIntaking()) {
					storage.runStorage(STORAGE_INTAKE_SPEED);
				} else {
					storage.runStorage(0);
				}
			}

		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		storage.runStorage(0);
	}
}
