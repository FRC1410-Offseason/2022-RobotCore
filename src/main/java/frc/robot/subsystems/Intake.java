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
    private boolean extended = false;

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
    public boolean getFlipperState() {
        return extended;
    }

    /**
     * Set the current state of the intake to extended
     */
    public void extend() {
        if (!extended) {
            setFlipper(Value.kForward);
            extended = true;
        }
    }

    /**
     * Set the state of the intake to retracted
     */
    public void retract() {
        if (extended) {
            setFlipper(Value.kReverse);
            extended = false;
        }
    }

    /**
     * Toggle the state of the intake
     */
    public void toggle() {
        if (extended) {
            retract();
        } else {
            extend();
        }
    }
}

