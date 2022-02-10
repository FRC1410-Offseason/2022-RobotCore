package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robotmap.Constants.DT;
import static frc.robotmap.IDs.*;

public class Winch extends SubsystemBase {

	//Winch motors
	private final VictorSPX leftMotor = new VictorSPX(WINCH_LEFT_MOTOR_ID);
	private final VictorSPX rightMotor = new VictorSPX(WINCH_RIGHT_MOTOR_ID);
	//Winch brake
	private final DoubleSolenoid lock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, WINCH_FWD, WINCH_BCK);

	//State variable to track state of locks
	private boolean lockEngaged = false;

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
		//Configure motors
		leftMotor.configFactoryDefault();
		rightMotor.configFactoryDefault();
		//Configure limit switch motor integration
		leftMotor.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteCANifier, LimitSwitchNormal.NormallyOpen, WINCH_LEFT_LIMIT_SWITCH_ID);
		leftMotor.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteCANifier, LimitSwitchNormal.NormallyOpen, WINCH_LEFT_LIMIT_SWITCH_ID);
		rightMotor.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteCANifier, LimitSwitchNormal.NormallyOpen, WINCH_RIGHT_LIMIT_SWITCH_ID);
		rightMotor.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteCANifier, LimitSwitchNormal.NormallyOpen, WINCH_RIGHT_LIMIT_SWITCH_ID);
		//Lock on startup
		lock();

		SmartDashboard.putData("Winch Piston", pistonSim);
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
	 *
	 * @param speed Speed from -1 to 1
	 */
	public void runWinch(double speed) {
		leftMotor.set(ControlMode.PercentOutput, speed);
		rightMotor.set(ControlMode.PercentOutput, speed);
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
