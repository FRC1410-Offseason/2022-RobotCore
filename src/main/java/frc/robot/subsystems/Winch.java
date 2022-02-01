package frc.robot.subsystems;

import static frc.robotmap.IDs.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase {
    //Winch motors
    private final CANSparkMax leftMotor = new CANSparkMax(WINCH_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(WINCH_RIGHT_MOTOR_ID, MotorType.kBrushless);
    //Winch Brakes
    private final DoubleSolenoid leftLock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, WINCH_L_FWD, WINCH_L_BCK);
    private final DoubleSolenoid rightLock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, WINCH_R_FWD, WINCH_R_BCK);

    //State variable to track state of locks
    private boolean lockState = false;

    public Winch() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
    }

    /**
     * Sets motor speeds
     * @param speed Speed from -1 to 1
     */
    public void runWinch(double speed){
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    /**
     * Set the state of the locks
     * @param state forward or backward
     */
    public void setLock(Value state) {
        leftLock.set(state);
        rightLock.set(state);
    }

    /**
     * Return the state of the locks
     * @return True / False -> Locked / Unlocked
     */
    public boolean getLockState() {
        return this.lockState;
    }

    /**
     * Set the state of the locks to locked
     */
    public void lock() {
        if (!this.lockState) {
            this.setLock(Value.kForward);
            this.lockState = true;
        }
    }

    /**
     * Set the state of the locks to unlocked
     */
    public void unlock() {
        if (this.lockState) {
            this.setLock(Value.kReverse);
            this.lockState = false;
        }
    }

    /**
     * Toggle the state of the locks
     */
    public void toggle() {
        if (this.lockState) {
            this.unlock();
        } else {
            this.lock();
        }
    }
}
