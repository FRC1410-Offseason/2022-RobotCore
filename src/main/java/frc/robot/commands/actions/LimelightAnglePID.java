package frc.robot.commands.actions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class LimelightAnglePID extends CommandBase {    
    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private static final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static final NetworkTable robotState = instance.getTable("Robot State");
    private double pidOutput = 0, kP = 0, kI = 0, kD = 0;
    private PIDController pid;

	public LimelightAnglePID(Limelight limelight, Drivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("LL Angle Init");
        kP = robotState.getEntry("Limelight Angle P Gain").getDouble(0);
        kI = robotState.getEntry("Limelight Angle I Gain").getDouble(0);
        kD = robotState.getEntry("Limelight Angle D Gain").getDouble(0);

		//Create a new PID controller with the constants from networktables
        pid = new PIDController(kP, kI, kD);

		// Set the target of the controller to 0, as that is the middle of the limelight's view
        pid.setSetpoint(0);
        
    }

    @Override
    public void execute() {        
		// Calculate an output from the controller using the current yaw angle from the target
		// If the limelight has a target, then we can set the drivetrain to the outputs from the controller and actually spin the robot
        
        if (limelight.hasTarget()) {
            pidOutput = pid.calculate(limelight.getYaw());
            System.out.println(pidOutput + "PID OUTPUT");
			drivetrain.tankDriveVolts(-pidOutput, pidOutput);
		} else drivetrain.tankDriveVolts(0, 0);
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("LL Angle Ended");
		drivetrain.tankDriveVolts(0, 0);
        if (!limelight.hasTarget()) System.out.println("No target");
	}

    @Override
    public boolean isFinished() {
		// If the current error from the target is within out acceptable margin, then we are done aligning and can shoot
        if (limelight.hasTarget()) return limelight.isYawAcceptable(144, limelight.getYaw());
        // if (limelight.hasTarget()) return Math.abs(limelight.getYaw()) < 2;
        else return true;
    }
}
