package frc.robot.commands.actions;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;


public class ResetShooterArmEncoderWithEntry extends CommandBase {

	private final ShooterArm shooterArm;
	private final NetworkTableEntry entry;

	public ResetShooterArmEncoderWithEntry(ShooterArm shooterArm, NetworkTableEntry entry) {
		this.shooterArm = shooterArm;
		this.entry = entry;
		addRequirements(this.shooterArm);
	}

	@Override
	public void initialize() {
		shooterArm.resetEncoder(entry.getDouble(51));
	}

	@Override
	public void execute() {

	}

	@Override
	public boolean isFinished() {
		// TODO: Make this return true when this Command no longer needs to run execute()
		return false;
	}

	@Override
	public void end(boolean interrupted) {

	}
}
