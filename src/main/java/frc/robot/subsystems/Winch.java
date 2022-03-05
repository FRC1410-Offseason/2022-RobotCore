package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.framework.subsystem.SubsystemBase;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Constants.*;

public class Winch extends SubsystemBase {

	private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
	private final NetworkTable table = instance.getTable("Winches");

	private final NetworkTableEntry leftLimit = table.getEntry("Left Limit Switch");
	private final NetworkTableEntry rightLimit = table.getEntry("Right Limit Switch");

	/**
	 * Motors
	 */
	private final VictorSPX leftMotor = new VictorSPX(WINCH_LEFT_MOTOR_ID);
	private final VictorSPX rightMotor = new VictorSPX(WINCH_RIGHT_MOTOR_ID);

	/**
	 * Solenoid for brake pistons
	 */
	private final DoubleSolenoid lock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, WINCH_FWD, WINCH_BCK);

	/**
	 * Limit switches to prevent demolition
	 */
	private final DigitalInput leftSwitch = new DigitalInput(WINCH_LEFT_LIMIT_SWITCH_ID);
	private final DigitalInput rightSwitch = new DigitalInput(WINCH_RIGHT_LIMIT_SWITCH_ID);

	/**
	 * Keeps track of whether the brake pistons are extended or not
	 */
	private boolean lockEngaged = false;

	public Winch() {
		// Configure motors
		leftMotor.configFactoryDefault();
		rightMotor.configFactoryDefault();

		leftMotor.setInverted(true);

		leftLimit.setBoolean(getLeftSwitch());
		rightLimit.setBoolean(getRightSwitch());
	}

	@Override
	public void periodic() {
		leftLimit.setBoolean(getLeftSwitch());
		rightLimit.setBoolean(getRightSwitch());
	}

	/**
	 * Sets motor speeds
	 * @param speed Speed from -1 to 1
	 */
	public void runWinch(double speed) {
		if (!getRightSwitch() && !getLeftSwitch()) {
			leftMotor.set(ControlMode.PercentOutput, speed * WINCH_LEFT_MOD);
			rightMotor.set(ControlMode.PercentOutput, speed);
		}
	}

	/**
	 * Return value of left limit switch
	 * @return True / False -> Triggered / Not triggered
	 */
	public boolean getLeftSwitch() {
		return leftSwitch.get();
	}

	/**
	 * Return value of right limit switch
	 * @return True / False -> Triggered / Not triggered
	 */
	public boolean getRightSwitch() {
		return rightSwitch.get();
	}

	/**
	 * Set the state of the locks
	 * @param state forward or backward
	 */
	public void setLock(Value state) {lock.set(state);}

	/**
	 * Return the state of the locks
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
