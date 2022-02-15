package frc.robot.commands.actions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.NetworkTables;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class LimelightAnglePID extends CommandBase {    
    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private PIDController pid;

	public LimelightAnglePID(Limelight limelight, Drivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        // addRequirements(drivetrain, limelight);
    }

    @Override
    public void initialize() {
        pid = new PIDController(NetworkTables.getLimelightAngleKP(), 
            NetworkTables.getLimelightAngleKI(), NetworkTables.getLimelightAngleKD());
        pid.setSetpoint(0);
    }

    @Override
    public void execute() {
		double pidOutput = pid.calculate(limelight.getYaw());
        double antifrictionOutput = Math.sqrt(pidOutput);
        if (limelight.getTarget() != null) drivetrain.tankDriveVolts(antifrictionOutput, -antifrictionOutput);
        else drivetrain.tankDriveVolts(0, 0);
    }
    
    @Override
    public void end(boolean interrupted) {
		drivetrain.tankDriveVolts(0, 0);
	}

    @Override
    public boolean isFinished() {
        if (limelight.getTarget() != null) {
            return Math.abs(limelight.getYaw()) < limelight.getAcceptableYaw(limelight.getDistanceToTarget());
        } else return false;
    }
}
