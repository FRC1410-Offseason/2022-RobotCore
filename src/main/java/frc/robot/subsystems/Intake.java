package frc.robot.subsystems;

import static frc.robotmap.IDs.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.framework.subsystem.SubsystemBase;

public class Intake extends SubsystemBase {
    //Flipping Solenoids
    private final DoubleSolenoid flipperLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, INTAKE_FLIPPER_L_FWD, INTAKE_FLIPPER_L_BCK);
    private final DoubleSolenoid flipperRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, INTAKE_FLIPPER_R_FWD, INTAKE_FLIPPER_R_BCK);
    //Internal state variable to keep track of flipper state
    private boolean flipperState = false;

    //Motor that runs the intake
    private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    public Intake() {
        intakeMotor.restoreFactoryDefaults();
    }

    /**
     * Set the speed of the intake
     * @param speed Speed from -1 to 1
     */
    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    /**
     * Returns the speed of the intake
     * @return Speed of motor
     */
    public double getSpeed() {
        return intakeMotor.get();
    }

    /**
     * Set the state of the intake
     * @param state either forward or reverse
     */
    public void setFlipper(Value state) {
        flipperLeft.set(state);
        flipperRight.set(state);
    }

    /**
     * Returns the current state of the intake
     * @return True / False -> Extended / Retracted
     */
    public boolean getState() {
        return this.flipperState;
    }

    /**
     * Set the current state of the intake to extended
     */
    public void extend() {
        if (!this.flipperState) {
            this.setFlipper(Value.kForward);
            this.flipperState = true;
        }
    }

    /**
     * Set the state of the intake to retracted
     */
    public void retract() {
        if (this.flipperState) {
            this.setFlipper(Value.kReverse);
            this.flipperState = false;
        }
    }

    /**
     * Toggle the state of the intake
     */
    public void toggle() {
        if (this.flipperState) {
            this.retract();
        } else {
            this.extend();
        }
    }
}

