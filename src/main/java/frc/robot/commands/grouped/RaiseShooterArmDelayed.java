package frc.robot.commands.grouped;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.actions.RaiseShooterArm;
import frc.robot.subsystems.ShooterArm;

import static frc.robotmap.Tuning.*;

public class RaiseShooterArmDelayed extends SequentialCommandGroup {

	public RaiseShooterArmDelayed(ShooterArm shooterArm) {

		addCommands (
				new WaitCommand(INTAKE_DEPLOYMENT_DELAY),
				new RaiseShooterArm(shooterArm)
		);
	}
}
