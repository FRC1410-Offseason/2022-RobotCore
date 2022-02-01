package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;


public class SetShooterArmAngle extends CommandBase {
    private final ShooterArm shooterArm;
    private final double targetAngle;

    public SetShooterArmAngle(ShooterArm shooterArm, double angle) {
        this.shooterArm = shooterArm;
        this.targetAngle = angle;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(shooterArm);
    }

    @Override
    public void initialize() {
        this.shooterArm.setTargetAngle(this.targetAngle);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }
}
