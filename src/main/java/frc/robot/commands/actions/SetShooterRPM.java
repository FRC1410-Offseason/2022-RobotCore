package frc.robot.commands.actions;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SetShooterRPM extends CommandBase {

	private final Shooter shooter;
	private final NetworkTableEntry RPM;

	public SetShooterRPM(Shooter shooter, NetworkTableEntry RPM) {
		this.shooter = shooter;
		this.RPM = RPM;
	}

	@Override
	public void initialize() {
		shooter.setSpeeds(RPM.getDouble(0));
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
