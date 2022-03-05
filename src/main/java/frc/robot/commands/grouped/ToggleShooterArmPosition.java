package frc.robot.commands.grouped;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.LowerShooterArm;
import frc.robot.commands.actions.RaiseShooterArm;
import frc.robot.subsystems.ShooterArm;

import java.util.ArrayList;

public class ToggleShooterArmPosition extends SequentialCommandGroup {

	public ToggleShooterArmPosition(ShooterArm shooterArm) {

		ArrayList<Command> toRun = new ArrayList<>();

		if (shooterArm.getLowerLimit()) {
			toRun.add(new RaiseShooterArm(shooterArm));
		} else {
			toRun.add(new LowerShooterArm(shooterArm));
		}

		addCommands();
	}
}
