package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.framework.subsystem.SubsystemBase;

import static frc.robotmap.Constants.*;
import static frc.robotmap.IDs.*;

public class Elevator extends SubsystemBase {

	//Elevator motors
	private final CANSparkMax leftMotor = new CANSparkMax(ELEVATOR_LEFT_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(ELEVATOR_RIGHT_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private double currentVoltage = 0;

	private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
	private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

	//Elevator Brakes
	private final DoubleSolenoid lock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ELEVATOR_FWD, ELEVATOR_BCK);

	//State variable to track state of locks
	private boolean locked = false;

	private final TrapezoidProfile.Constraints constraints =
			new TrapezoidProfile.Constraints(
					ELEVATOR_MAX_VEL,
					ELEVATOR_MAX_ACCEL
			);

//	private final LinearSystem<N2, N1, N1> plant =
//			LinearSystemId.identifyPositionSystem(
//					ELEVATOR_KV,
//					ELEVATOR_KA
//			);

	private final LinearSystem<N2, N1, N1> plant =
			LinearSystemId.createElevatorSystem(
					DCMotor.getNEO(1),
					ELEVATOR_MASS,
					Units.inchesToMeters(1),
					GEAR_RATIO
			);

	private final KalmanFilter<N2, N1, N1> observer =
			new KalmanFilter<>(
					Nat.N2(),
					Nat.N1(),
					plant,
					VecBuilder.fill(ELEVATOR_POS_CONFIDENCE, ELEVATOR_VEL_CONFIDENCE),
					VecBuilder.fill(ELEVATOR_ENC_CONFIDENCE),
					DT
			);

	private final LinearQuadraticRegulator<N2, N1, N1> controller =
			new LinearQuadraticRegulator<>(
					plant,
					VecBuilder.fill(ELEVATOR_POS_TOLERANCE, ELEVATOR_VEL_TOLERANCE),
					VecBuilder.fill(ELEVATOR_CTRL_TOLERANCE),
					DT
			);

	private final LinearSystemLoop<N2, N1, N1> loop =
			new LinearSystemLoop<>(plant, controller, observer, ELEVATOR_CTRL_TOLERANCE, DT);

	private final LinearSystemSim<N2, N1, N1> sim = new LinearSystemSim<>(plant);

	private final Mechanism2d simWidget = new Mechanism2d(20, 50);
	private final MechanismRoot2d widgetRoot = simWidget.getRoot("Elevator Root", 10, 0);
	private final MechanismLigament2d elevatorSim =
			widgetRoot.append(
					new MechanismLigament2d(
							"Elevator",
							0,
							90
					)
			);

	/**
	 * Used for the piston widget in the simulation gui
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


	public Elevator() {
		leftMotor.restoreFactoryDefaults();
		rightMotor.restoreFactoryDefaults();

		leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		leftEncoder.setPositionConversionFactor(ELEVATOR_METERS_PER_REV);
		rightEncoder.setPositionConversionFactor(ELEVATOR_METERS_PER_REV);

		SmartDashboard.putData("Elevator Sim", simWidget);
		SmartDashboard.putData("Piston Sim", pistonSim);
	}

	@Override
	public void periodic() {

	}

	@Override
	public void simulationPeriodic() {
		sim.setInput(currentVoltage);

		if (lock.get() == Value.kForward) {
			pistonInnards.setLength(40);
			sim.update(0);
		} else {
			pistonInnards.setLength(0);
			sim.update(DT);
		}

		leftEncoder.setPosition(sim.getOutput(0));
		rightEncoder.setPosition(sim.getOutput(0));

		elevatorSim.setLength(loop.getObserver().getXhat(0) * 23);
	}

	public LinearSystemLoop<N2, N1, N1> getLoop() {
		return loop;
	}

	public TrapezoidProfile.Constraints getConstraints() {
		return constraints;
	}

	public double getEncoderPosition() {
		return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
	}

	public double getEncoderVelocity() {
		return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
	}

	public void setVoltage(double voltage) {
		currentVoltage = voltage;
		leftMotor.setVoltage(voltage);
		rightMotor.setVoltage(voltage);
	}

	public void set(double speed) {
		System.out.println("Speed of motors set to" + speed);
		leftMotor.set(speed);
		rightMotor.set(speed);
	}

	public double getSpeed() {
		return (leftMotor.get() + rightMotor.get()) / 2;
	}

	public double getCurrentVoltage() {
		return currentVoltage;
	}

	/**
	 * Sets left motor speed
	 *
	 * @param speed Speed from -1 to 1
	 */
	public void runLeftElevator(double speed) {
		leftMotor.set(speed);
	}

	/**
	 * Sets right motor speed
	 *
	 * @param speed Speed from -1 to 1
	 */
	public void runRightElevator(double speed) {
		rightMotor.set(speed);
	}

	/**
	 * Gets height of left elevator
	 *
	 * @return Height of left elevator
	 */
	public double getHeightLeft() {
		leftMotor.getEncoder();
		return leftMotor.get() * GEAR_RATIO;
	}

	/**
	 * Gets height of right elevator
	 *
	 * @return Height of right elevator
	 */
	public double getHeightRight() {
		rightMotor.getEncoder();
		return rightMotor.get() * GEAR_RATIO;
	}

	/**
	 * Set the state of the locks
	 *
	 * @param state forward or backward
	 */
	public void setLock(Value state) {
		lock.set(state);
	}

	/**
	 * Return the state of the locks
	 *
	 * @return True / False -> Locked / Unlocked
	 */
	public boolean getLocked() {
		return this.locked;
	}

	/**
	 * Set the state of the locks to locked
	 */
	public void lock() {
		if (!this.locked) {
			this.setLock(Value.kForward);
			this.locked = true;
		}
	}

	/**
	 * Set the state of the locks to unlocked
	 */
	public void unlock() {
		if (this.locked) {
			this.setLock(Value.kReverse);
			this.locked = false;
		}
	}

	/**
	 * Toggle the state of the locks
	 */
	public void toggle() {
		if (this.locked) {
			this.unlock();
		} else {
			this.lock();
		}
	}
}
