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
    private double pidOutput;

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
        pidOutput = pid.calculate(limelight.getYaw());
        if (limelight.getTarget() != null) {
            drivetrain.tankDriveVolts(pidOutput, -pidOutput); // maybe needs to be inverse
        } else drivetrain.tankDriveVolts(0, 0);
    }
    
    @Override
    public void end(boolean interrupted) {drivetrain.tankDriveVolts(0, 0);}

    @Override
    public boolean isFinished() {
        if (limelight.getTarget() != null) {
            return Math.abs(limelight.getYaw()) < limelight.getAcceptableYaw(limelight.getDistanceToTarget());  
        } else return false;
    }
}
