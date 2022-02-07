package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robotmap.IDs.*;

public class Winch extends SubsystemBase {

	//Winch motors
	private final CANSparkMax leftMotor = new CANSparkMax(WINCH_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(WINCH_RIGHT_MOTOR_ID, MotorType.kBrushless);
	//Winch Brakes
	private final DoubleSolenoid lock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, WINCH_FWD, WINCH_BCK);

	//State variable to track state of locks
	private boolean lockEngaged = false;

	public Winch() {
		leftMotor.restoreFactoryDefaults();
		rightMotor.restoreFactoryDefaults();
	}

	/**
	 * Sets motor speeds
	 *
	 * @param speed Speed from -1 to 1
	 */
	public void runWinch(double speed) {
		leftMotor.set(speed);
		rightMotor.set(speed);
	}

	/**
	 * Set the state of the locks
	 *
	 * @param state forward or backward
	 */
	public void setLock(Value state) {lock.set(state);}

	/**
	 * Return the state of the locks
	 *
	 * @return True / False -> Locked / Unlocked
	 */
	public boolean isLocked() {
		return lockEngaged;
	}

	/**
	 * Set the state of the locks to locked
	 */
	public void lock() {
		if (!lockEngaged) {
			setLock(Value.kForward);
			lockEngaged = true;
		}
	}

	/**
	 * Set the state of the locks to unlocked
	 */
	public void unlock() {
		if (lockEngaged) {
			setLock(Value.kReverse);
			lockEngaged = false;
		}
	}

	/**
	 * Toggle the state of the locks
	 */
	public void toggle() {
		if (lockEngaged) {
			unlock();
		} else {
			lock();
		}
	}
}
