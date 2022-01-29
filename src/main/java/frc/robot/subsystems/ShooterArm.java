package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robotmap.IDs.*;

public class ShooterArm extends SubsystemBase {
    //Motors that run shooter arm
    private final CANSparkMax shooterArmLeft = new CANSparkMax(SHOOTER_ARM_L_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax shooterArmRight = new CANSparkMax(SHOOTER_ARM_R_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    public ShooterArm() {
        shooterArmLeft.restoreFactoryDefaults();
        shooterArmRight.restoreFactoryDefaults();
    }

    /**
     * Sets the speed of the shooter arm
     * @param speed Speed from -1 to 1
     */
    public void setSpeed(double speed) {
        shooterArmLeft.set(speed);
        shooterArmRight.set(speed);
    }

    /**
     * Return the speed of the shooter arm
     * @return Speed of motors
     */
    public double getSpeed() {
        return (shooterArmLeft.get() + shooterArmRight.get()) / 2;
    }
}
