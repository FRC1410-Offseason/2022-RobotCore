package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robotmap.IDs.*;

public class ShooterArm extends SubsystemBase {
    //Motors that run shooter arm
    private final CANSparkMax shooterArmLeftMotor = new CANSparkMax(SHOOTER_ARM_L_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    private final CANSparkMax shooterArmRightMotor = new CANSparkMax(SHOOTER_ARM_R_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    //Locking Solenoid
    private final DoubleSolenoid lock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SHOOTER_ARM_LOCK_FWD, SHOOTER_ARM_LOCK_BCK);
    //State variable to track state of lock
    private boolean lockState = false;

    public ShooterArm() {
        shooterArmLeftMotor.restoreFactoryDefaults();
        shooterArmRightMotor.restoreFactoryDefaults();
    }

    /**
     * Sets the speed of the shooter arm
     * @param speed Speed from -1 to 1
     */
    public void setSpeed(double speed) {
        shooterArmLeftMotor.set(speed);
        shooterArmRightMotor.set(speed);
    }

    /**
     * Return the speed of the shooter arm
     * @return Speed of motors
     */
    public double getSpeed() {
        return (shooterArmLeftMotor.get() + shooterArmRightMotor.get()) / 2;
    }

    /**
     * Set the state of the lock
     * @param state forward or backward
     */
    public void setLock(Value state) {
        lock.set(state);
    }

    /**
     * Return the state of the lock
     * @return True / False -> Locked / Unlocked
     */
    public boolean getState() {
        return this.lockState;
    }

    /**
     * Set the state of the lock to locked
     */
    public void lock() {
        if (!this.lockState) {
            this.setLock(Value.kForward);
            this.lockState = true;
        }
    }

    /**
     * Set the state of the lock to unlocked
     */
    public void unlock() {
        if (this.lockState) {
            this.setLock(Value.kReverse);
            this.lockState = false;
        }
    }

    /**
     * Toggle the state of the lock
     */
    public void toggle() {
        if (this.lockState) {
            this.unlock();
        } else {
            this.lock();
        }
    }
}
