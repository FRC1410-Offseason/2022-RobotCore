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
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

	/**
	 * Used for the simulation widgets, not currently functional
	 */
	private final Mechanism2d pistonSim = new Mechanism2d(60, 60);
	private final MechanismRoot2d pistonSimRoot = pistonSim.getRoot("Piston", 30, 10);
	private final MechanismLigament2d piston =
			pistonSimRoot.append(
					new MechanismLigament2d(
							"Piston Casing",
							20,
							90
					)
			);
	private final MechanismLigament2d pistonInnards =
			pistonSimRoot.append(
					new MechanismLigament2d(
							"Piston",
							20,
							90,
							4,
							new Color8Bit(Color.kRed)
					)
			);

	public Winch() {
		// Configure motors
		leftMotor.configFactoryDefault();
		rightMotor.configFactoryDefault();

		leftMotor.setInverted(true);

		// Configure limit switch motor integration
		// TODO: Make sure this actually works like we expect it does
		//53.06 differnce in gearboxes?
		// Lock on startup
//		lock();

		SmartDashboard.putData("Winch Piston", pistonSim);

		leftLimit.setBoolean(getLeftSwitch());
		rightLimit.setBoolean(getRightSwitch());
	}

	@Override
	public void periodic() {
		leftLimit.setBoolean(getLeftSwitch());
		rightLimit.setBoolean(getRightSwitch());
	}

	@Override
	public void simulationPeriodic() {
		if (lock.get() == Value.kForward) {
			pistonInnards.setLength(40);
		} else {
			pistonInnards.setLength(0);
		}
	}

	/**
	 * Sets motor speeds
	 * @param speed Speed from -1 to 1
	 */
	public void runWinch(double speed) {
		// TODO: Make limit switches work
//		if (!getLeftSwitch() && !getRightSwitch()) {
//			leftMotor.set(ControlMode.PercentOutput, speed * WINCH_LEFT_MOD);
//			rightMotor.set(ControlMode.PercentOutput, speed);
//		} else {
//			leftMotor.set(ControlMode.PercentOutput, 0);
//			rightMotor.set(ControlMode.PercentOutput, 0);
//		}
		leftMotor.set(ControlMode.PercentOutput, speed * WINCH_LEFT_MOD);
		rightMotor.set(ControlMode.PercentOutput, speed);
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
