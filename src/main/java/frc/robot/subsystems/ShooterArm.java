package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
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
import static frc.robotmap.Tuning.*;

public class ShooterArm extends SubsystemBase {

	/**
	 * Motors
	 */
	private final CANSparkMax leftMotor = new CANSparkMax(SHOOTER_ARM_L_MOTOR, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(SHOOTER_ARM_R_MOTOR, MotorType.kBrushless);

	/**
	 * Grabbing the encoder objects from the motors
	 */
	private final WPI_TalonSRX encoderMotor;
	private double encoderOffset = 0;

	private final PIDController controller = new PIDController(SA_P, SA_I, SA_D); // I am also sad

	private double target = 19;

	/**
	 * For the physical brake piston on the mechanism
	 */
	private final DoubleSolenoid brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SHOOTER_ARM_LOCK_FWD, SHOOTER_ARM_LOCK_BCK);

	//<editor-fold desc="Sim Stuff" defaultstate="collapsed">

	/**
	 * Used for simulating the mechanism
	 */
	private final SingleJointedArmSim sim =
			new SingleJointedArmSim(
					DCMotor.getNEO(2),
					SHOOTER_ARM_GEARING,
					SingleJointedArmSim.estimateMOI(SHOOTER_ARM_LENGTH, SHOOTER_ARM_MASS),
					SHOOTER_ARM_LENGTH,
					Units.degreesToRadians(SHOOTER_ARM_RESTING_ANGLE),
					Units.degreesToRadians(SHOOTER_ARM_MAX_ANGLE),
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
	//</editor-fold>

	public ShooterArm(WPI_TalonSRX encoderMotor) {
		// Reset the controllers
		leftMotor.restoreFactoryDefaults();
		rightMotor.restoreFactoryDefaults();

		// Set them to use brake mode
		leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		leftMotor.setInverted(true);
		rightMotor.setInverted(false);

		this.encoderMotor = encoderMotor;

		// TODO: Find out if this needs to be relative or absolute for the encoder type
		encoderMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

		// Send the arm widget to Smart Dashboard
		SmartDashboard.putData("Arm Sim", simMech);
		tower.setColor(new Color8Bit(Color.kBlue));

		// Send the piston widget to Smart Dashboard
		SmartDashboard.putData("Shooter Arm Piston", pistonSim);

		// Default state for the brake piston is extended
		setBrake();
	}

	@Override
	// Broken atm, even more so now
	public void simulationPeriodic() {
		// Update the arm widget if the brake is extended
		if (getBrakeState()) {
			pistonInnards.setLength(40);
		} else {
			pistonInnards.setLength(0);
		}

		// Set inputs to the simulator
		sim.setInputVoltage(currentVoltage);

		// Update the sim (default time is 20 ms)
		sim.update(DT50HZ);
	}

	public void setTarget(double value) {
		target = MathUtil.clamp(value, SHOOTER_ARM_RESTING_ANGLE, SHOOTER_ARM_MAX_ANGLE);
	}

	public double getTarget() {
		return target;
	}

	/**
	 * Get the current position of the mechanism
	 * @return average encoder position in radians
	 */
	public double getEncoderPosition() {
		return (encoderMotor.getSelectedSensorPosition() * 360 / 4096) - encoderOffset;
	}

	public void runPIDExecute() {
		double output = controller.calculate(getEncoderPosition(), target);
		leftMotor.set(output);
		rightMotor.set(output);
	}

	/**
	 * Set the voltage of the motors
	 * @param voltage desired voltage
	 */
	public void setVoltage(double voltage) {
		currentVoltage = voltage;
		leftMotor.setVoltage(voltage);
		rightMotor.setVoltage(voltage);
	}

	public boolean isAtTarget() {
		return Math.abs(getEncoderPosition() - target) < SHOOTER_ARM_IS_FINISHED;
	}

	/**
	 * Get the state of the brake piston
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

