package frc.robot.subsystems;

import static frc.robotmap.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    //State space model
    LinearSystem<N2, N1, N1> armPlant = LinearSystemId.identifyPositionSystem(SHOOTER_ARM_KV, SHOOTER_ARM_KA);

    //Observer: Kalman Filter
    private final KalmanFilter<N2, N1, N1> armObserver =
            new KalmanFilter<>(
                    Nat.N2(),
                    Nat.N1(),
                    armPlant,
                    VecBuilder.fill(SHOOTER_ARM_POS_CONFIDENCE, SHOOTER_ARM_VEL_CONFIDENCE),
                    VecBuilder.fill(SHOOTER_ARM_ENC_CONFIDENCE),
                    DT
            );

    //Controller: Linear Quadratic Regulator
    private final LinearQuadraticRegulator<N2, N1, N1> armController =
            new LinearQuadraticRegulator<>(
              armPlant,
              VecBuilder.fill(SHOOTER_ARM_POS_TOLERANCE, SHOOTER_ARM_VEL_TOLERANCE),
              VecBuilder.fill(SHOOTER_ARM_CTRL_TOLERANCE),
              DT
            );

    private final TrapezoidProfile.Constraints armConstraints =
            new TrapezoidProfile.Constraints(
                SHOOTER_ARM_MAX_VELOCITY,
                SHOOTER_ARM_MAX_ACCEL
            );


    private final LinearSystemLoop<N2, N1, N1> armLoop =
            new LinearSystemLoop<>(armPlant, armController, armObserver, SHOOTER_ARM_MAX_VOLTAGE, DT);


    public ShooterArm() {
        //Reset motors to factory default
        shooterArmLeftMotor.restoreFactoryDefaults();
        shooterArmRightMotor.restoreFactoryDefaults();

        //Reset control loop to known state
        armLoop.reset(VecBuilder.fill(0, 0));
    }

    /**
     * Get the constraints of the system
     * @return the constraints of the arm system
     */
    public TrapezoidProfile.Constraints getConstraints() {
        return this.armConstraints;
    }

    /**
     * Returns the control loop for the arm subsystem to be used in commands
     * @return a LinearSystemLoop object that represents the arm subsystem
     */
    public LinearSystemLoop<N2, N1, N1> getControlLoop() {
        return this.armLoop;
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
    public boolean getLockState() {
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
