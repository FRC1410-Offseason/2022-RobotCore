package frc.robot.commands.grouped;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.*;
import frc.robot.subsystems.IntakeFlipper;

import java.util.ArrayList;

import static frc.robotmap.Tuning.*;

public class ToggleIntakePosition extends SequentialCommandGroup {

	public ToggleIntakePosition(IntakeFlipper intakeFlipper) {

		ArrayList<Command> toRun = new ArrayList<>();


		addCommands(toRun.toArray(Command[]::new));
	}
}
