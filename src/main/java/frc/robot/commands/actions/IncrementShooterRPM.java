package frc.robot.commands.actions;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NetworkTables;
import frc.robot.subsystems.Shooter;

public class IncrementShooterRPM extends CommandBase {

	private final Shooter shooter;
	private final NetworkTableEntry RPM;
	private final double increment;

	public IncrementShooterRPM(Shooter shooter, NetworkTableEntry RPM, double increment) {
		this.shooter = shooter;
		this.RPM = RPM;
		this.increment = increment;
	}

	@Override
	public void initialize() {
		shooter.setSpeeds(RPM.getDouble(0) + increment);
		NetworkTables.setShooterLowRPM(RPM.getDouble(0) + increment);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}

