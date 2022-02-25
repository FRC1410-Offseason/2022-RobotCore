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
		//Create a new PID controller with the constants from networktables
        pid = new PIDController(NetworkTables.getLimelightAngleKP(), 
            NetworkTables.getLimelightAngleKI(), NetworkTables.getLimelightAngleKD());

		// Set the target of the controller to 0, as that is the middle of the limelight's view
        pid.setSetpoint(0);
    }

    @Override
    public void execute() {
		// Calculate an output from the controller using the current yaw angle from the target
		double pidOutput = pid.calculate(limelight.getYaw());

		// If the limelight has a target, then we can set the drivetrain to the outputs from the controller and actually spin the robot
        if (limelight.getTarget() != null) {
			drivetrain.tankDriveVolts(-pidOutput, pidOutput);
		} else {
			//If not then we don't actually want to move
			drivetrain.tankDriveVolts(0, 0);
		}
    }
    
    @Override
    public void end(boolean interrupted) {
		drivetrain.tankDriveVolts(0, 0);
	}

    @Override
    public boolean isFinished() {
		// If the current error from the target is within out acceptable margin, then we are done aligning and can shoot
        if (limelight.getTarget() != null) {
            return Math.abs(limelight.getYaw()) < limelight.getAcceptableYaw(limelight.getDistanceToTarget());
        } else {
			return false;
		}
    }
}
