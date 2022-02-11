package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.framework.subsystem.SubsystemBase;

import static frc.robotmap.Constants.*;
import static frc.robotmap.IDs.*;

public class ShooterArm extends SubsystemBase {

	/**
	 * Motors
	 */
	private final CANSparkMax leftMotor = new CANSparkMax(SHOOTER_ARM_L_MOTOR, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(SHOOTER_ARM_R_MOTOR, MotorType.kBrushless);
	/**
	 * Grabbing the encoder objects from the motors
	 */
	private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
	private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
	/**
	 * For the physical brake piston on the mechanism
	 */
	private final DoubleSolenoid brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SHOOTER_ARM_LOCK_FWD, SHOOTER_ARM_LOCK_BCK);

	/**
	 * State space model of the mechanism
	 */
	private final LinearSystem<N2, N1, N1> plant =
			LinearSystemId.createSingleJointedArmSystem(
					DCMotor.getNEO(2),
					SingleJointedArmSim.estimateMOI(SHOOTER_ARM_LENGTH, SHOOTER_ARM_MASS),
					125
			);
	/**
	 * Used to filter inputs from the encoders, part of the state space loop
	 */
	private final KalmanFilter<N2, N1, N1> observer =
			new KalmanFilter<>(
					Nat.N2(),
					Nat.N1(),
					plant,
					VecBuilder.fill(SHOOTER_ARM_POS_CONFIDENCE, SHOOTER_ARM_VEL_CONFIDENCE),
					VecBuilder.fill(SHOOTER_ARM_ENC_CONFIDENCE),
					DT
			);
	/**
	 * Controller for the mechanism, part of the state space loop
	 */
	private final LinearQuadraticRegulator<N2, N1, N1> controller =
			new LinearQuadraticRegulator<>(
					plant,
					VecBuilder.fill(SHOOTER_ARM_POS_TOLERANCE, SHOOTER_ARM_VEL_TOLERANCE),
					VecBuilder.fill(SHOOTER_ARM_CTRL_TOLERANCE),
					DT
			);
	/**
	 * Pulls all the different state space parts into one loop to make things easier
	 */
	private final LinearSystemLoop<N2, N1, N1> loop =
			new LinearSystemLoop<>(plant, controller, observer, SHOOTER_ARM_MAX_VOLTAGE, DT);
	/**
	 * Constraints for the control
	 */
	private final TrapezoidProfile.Constraints constraints =
			new TrapezoidProfile.Constraints(
					Units.degreesToRadians(SHOOTER_ARM_MAX_VELOCITY),
					Units.degreesToRadians(SHOOTER_ARM_MAX_ACCEL)
			);

	private TrapezoidProfile.State goal;
	private TrapezoidProfile.State lpr;

	/**
	 * Used for simulating the mechanism
	 */
	private final SingleJointedArmSim sim =
			new SingleJointedArmSim(
					DCMotor.getNEO(2),
					SHOOTER_ARM_GEARING,
					SingleJointedArmSim.estimateMOI(SHOOTER_ARM_LENGTH, SHOOTER_ARM_MASS),
					SHOOTER_ARM_LENGTH,
					Units.degreesToRadians(SHOOTER_ARM_MIN_ROT),
					Units.degreesToRadians(SHOOTER_ARM_MAX_ROT),
					SHOOTER_ARM_MASS,
					false,
					null
			);
	/**
	 * For the widget in the simulation gui
	 */
	private final Mechanism2d simMech = new Mechanism2d(60, 60);
	private final MechanismRoot2d simPivot = simMech.getRoot("Pivot", 10, 10);
	private final MechanismLigament2d tower = simPivot.append(new MechanismLigament2d("Tower", 30, 0));
	private final MechanismLigament2d arm = simPivot.append(
			new MechanismLigament2d(
					"Arm",
					30,
					Units.radiansToDegrees(sim.getAngleRads()),
					6,
					new Color8Bit(Color.kYellow)
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
	/**
	 * Used for the simulation, because for some reason it's impossible to get the voltage that a motor is running at
	 */
	private double currentVoltage = 0;

	public ShooterArm() {
		//Reset the controllers
		leftMotor.restoreFactoryDefaults();
		rightMotor.restoreFactoryDefaults();
		//Set them to use brake mode
		leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		//Set the internal conversions of the motors so that they report in radians
		leftEncoder.setPositionConversionFactor(Math.PI * 2);
		rightEncoder.setPositionConversionFactor(Math.PI * 2);
		leftEncoder.setVelocityConversionFactor(Math.PI * 2);
		rightEncoder.setVelocityConversionFactor(Math.PI * 2);

		//Send the arm widget to Smart Dashboard
		SmartDashboard.putData("Arm Sim", simMech);
		tower.setColor(new Color8Bit(Color.kBlue));

		//Send the piston widget to Smart Dashboard
		SmartDashboard.putData("Shooter Arm Piston", pistonSim);

		//Default state for the brake piston is extended
		setBrake();

		//Reset the state space loop to a known position
		loop.reset(VecBuilder.fill(getEncoderPosition(), getEncoderVelocity()));
		lpr = new TrapezoidProfile.State(getEncoderPosition(), getEncoderVelocity());
		goal = new TrapezoidProfile.State(0, 0);
	}

	@Override
	public void periodic() {
		if (Math.abs(loop.getXHat(0) - goal.position) < Units.degreesToRadians(SHOOTER_ARM_IS_FINISHED_THRESHOLD)) {
			setBrake();
			setVoltage(0);
		} else {
			releaseBrake();
			lpr = (new TrapezoidProfile(constraints, goal, lpr)).calculate(DT);
			loop.setNextR(lpr.position, lpr.velocity);
			loop.correct(VecBuilder.fill(getEncoderPosition()));
			loop.predict(DT);
			setVoltage(loop.getU(0));
		}
	}

	@Override
	public void simulationPeriodic() {
		//Update the arm widget if the brake is extended
		if (getBrakeState()) {
			pistonInnards.setLength(40);
		} else {
			pistonInnards.setLength(0);
		}

		//Set inputs to the simulator
		sim.setInputVoltage(currentVoltage);

		//Update the sim (default time is 20 ms)
		sim.update(DT);

		//Update the position of the encoders from the sim
		leftEncoder.setPosition(sim.getAngleRads());
		rightEncoder.setPosition(sim.getAngleRads());

		//Set the angle of the arm widget from the kalman filter
		arm.setAngle(Units.radiansToDegrees(loop.getObserver().getXhat(0)));
	}

	/**
	 * Get the current position of the mechanism
	 *
	 * @return average encoder position in radians
	 */
	public double getEncoderPosition() {
		return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
	}

	/**
	 * Get the current velocity of the encoders
	 *
	 * @return average encoder velocity in radians per second
	 */
	public double getEncoderVelocity() {
		return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
	}

	/**
	 * Get the state space loop
	 *
	 * @return a linear system loop that contains the plant, observer, and controller for the mechanism
	 */
	public LinearSystemLoop<N2, N1, N1> getLoop() {
		return loop;
	}

	/**
	 * Get the physical constraints of the mechanism
	 *
	 * @return contains the maximum velocity and acceleration of the mechanism
	 */
	public TrapezoidProfile.Constraints getConstraints() {
		return constraints;
	}

	public void setGoal(double angle) {
		goal = new TrapezoidProfile.State(Units.degreesToRadians(angle), 0);
	}

	public TrapezoidProfile.State getGoal() {
		return goal;
	}

	/**
	 * Set the voltage of the motors
	 *
	 * @param voltage desired voltage
	 */
	public void setVoltage(double voltage) {
		currentVoltage = voltage;
		leftMotor.setVoltage(voltage);
		rightMotor.setVoltage(voltage);
	}

	/**
	 * Get the state of the brake piston
	 *
	 * @return false -> retracted / true -> extended
	 */
	public boolean getBrakeState() {
		return brake.get() == Value.kForward;
	}

	/**
	 * Set the state of the brake piston to extended
	 */
	public void setBrake() {
		brake.set(Value.kForward);
	}

	/**
	 * Set the state of the brake piston to retracted
	 */
	public void releaseBrake() {
		brake.set(Value.kReverse);
	}
}

