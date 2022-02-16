package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.framework.subsystem.SubsystemBase;

import static frc.robotmap.IDs.*;

public class Intake extends SubsystemBase {

	//Flipping Solenoids
	// TODO: This is probably going to need to be changed to a talon because the intake is no longer actuated with a piston
	private final DoubleSolenoid flipper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, INTAKE_FLIPPER_FWD, INTAKE_FLIPPER_BCK);

	// Motor that controls the flipping action
	private final TalonSRX flipperMotor = new TalonSRX(INTAKE_FLIPPER_MOTOR);

	//Motor that runs the intake
	private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

	//Internal state variable to keep track of flipper state
	private boolean extended = false;

	public Intake() {
		intakeMotor.restoreFactoryDefaults();
	}

	/**
	 * Returns the speed of the intake
	 *
	 * @return Speed of motor
	 */
	public double getSpeed() {
		return intakeMotor.get();
	}

	/**
	 * Set the speed of the intake
	 *
	 * @param speed Speed from -1 to 1
	 */
	public void setSpeed(double speed) {
		intakeMotor.set(speed);
	}

	/**
	 * Set the state of the intake
	 *
	 * @param state either forward or reverse
	 */
	public void setFlipper(Value state) {flipper.set(state);}

	/**
	 * Returns the current state of the intake
	 *
	 * @return True / False -> Extended / Retracted
	 */
	public boolean getExtended() {
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

