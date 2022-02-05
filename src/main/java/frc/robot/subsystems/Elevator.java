package frc.robot.subsystems;

import static frc.robotmap.IDs.*;

import static frc.robotmap.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    //Elevator motors
    private final CANSparkMax leftMotor = new CANSparkMax(ELEVATOR_LEFT_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(ELEVATOR_RIGHT_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    //Elevator Brakes
    private final DoubleSolenoid leftLock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ELEVATOR_L_FWD, ELEVATOR_L_BCK);
    private final DoubleSolenoid rightLock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ELEVATOR_R_FWD, ELEVATOR_R_BCK);

    //State variable to track state of locks
    private boolean lockState = false;

    public Elevator(){
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
    }

    /**
     * Sets left motor speed
     * @param speed Speed from -1 to 1
     */
    public void runLeftElevator(double speed){
        leftMotor.set(speed);
    }

    /**
     * Sets right motor speed
     * @param speed Speed from -1 to 1
     */
    public void runRightElevator(double speed){
        rightMotor.set(speed);
    }

    /**
     * Gets height of left elevator
     * @return Height of left elevator
     */
    public double getHeightLeft(){
        leftMotor.getEncoder();
        return leftMotor.get() * GEAR_RATIO;
    }

    /**
     * Gets height of right elevator
     * @return Height of right elevator
     */
    public double getHeightRight(){
        rightMotor.getEncoder();
        return rightMotor.get() * GEAR_RATIO;
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
