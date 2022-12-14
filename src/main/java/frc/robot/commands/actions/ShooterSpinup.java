package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import static frc.robotmap.Constants.*;

public class ShooterSpinup extends CommandBase {
    private final Shooter shooter;
    private double RPM;

    public ShooterSpinup(Shooter shooter, double RPM) {
        this.shooter = shooter;
        this.RPM = RPM;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setSpeeds(RPM);
    }

    @Override
    public boolean isFinished() {
        if (shooter.isAtTarget()) System.out.println("READY TO SHOOT");
        return shooter.isAtTarget();
    }
}
