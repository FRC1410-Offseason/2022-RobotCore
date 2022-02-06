package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class SetShooterRPM extends CommandBase {

	private final Shooter shooter;
	private final double RPM;

	public SetShooterRPM(Shooter shooter, double RPM) {
		this.shooter = shooter;
		this.RPM = RPM;
		// addRequirements(shooter);
	}

	@Override
	public void initialize() {
		shooter.setSpeeds(RPM);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
