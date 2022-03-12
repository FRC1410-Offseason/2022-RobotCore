package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.framework.subsystem.SubsystemBase;

import static frc.robotmap.Constants.*;
import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.*;

public class ShooterArm extends SubsystemBase {

	private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
	private final NetworkTable table = instance.getTable("Shooter Arm");

	private final NetworkTableEntry encoderPos = table.getEntry("Encoder Pos");
	private final NetworkTableEntry goalNT = table.getEntry("Goal");
	private final NetworkTableEntry controllerOutput = table.getEntry("Controller Output");
	private final NetworkTableEntry currentError = table.getEntry("Error");
	private final NetworkTableEntry rawEncoderPos = table.getEntry("Raw Encoder");

	private final NetworkTableEntry upperLimitNT = table.getEntry("Upper Limit");
	private final NetworkTableEntry lowerLimitNT = table.getEntry("Lower Limit");

	/**
	 * Motors
	 */
	private final CANSparkMax leftMotor = new CANSparkMax(SHOOTER_ARM_L_MOTOR, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(SHOOTER_ARM_R_MOTOR, MotorType.kBrushless);

	/**
	 * Grabbing the encoder objects from the motors
	 */
	private final WPI_TalonSRX encoderMotor;

	private final DigitalInput upperLimit = new DigitalInput(8);
	private final DigitalInput lowerLimit = new DigitalInput(1);

	private boolean manualControl = true;

	private boolean shouldRunUp = true;
	private boolean shouldRunDown = true;

	private final PIDController PID = new PIDController(SA_P, SA_I, SA_D); // I am also sad

	private double goal = SHOOTER_ARM_MAX_ANGLE;

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

		// Send the arm widget to Smart Dashboard
		SmartDashboard.putData("Arm Sim", simMech);
		tower.setColor(new Color8Bit(Color.kBlue));

		encoderPos.setDouble(0);
		goalNT.setDouble(0);

		controllerOutput.setDouble(0);
		rawEncoderPos.setDouble(0);

		upperLimitNT.setBoolean(false);
		lowerLimitNT.setBoolean(false);
	}

	@Override
	public void periodic() {
		encoderPos.setDouble(getEncoderPosition());
		goalNT.setDouble(goal);
		currentError.setDouble(PID.getPositionError());
		rawEncoderPos.setDouble(encoderMotor.getSelectedSensorPosition());

		upperLimitNT.setBoolean(upperLimit.get());
		lowerLimitNT.setBoolean(lowerLimit.get());

		if ((leftMotor.get() < 0 && lowerLimit.get())) {
			shouldRunDown = false;
			leftMotor.set(0);
			rightMotor.set(0);
		} else {
			shouldRunDown = true;
		}

		if ((leftMotor.get() > 0 && upperLimit.get())) {
			shouldRunUp = false;
			leftMotor.set(0);
			rightMotor.set(0);
		} else {
			shouldRunUp = true;
		}
	}

	public boolean shouldRunUp() {
		return shouldRunUp;
	}

	public boolean shouldRunDown() {
		return shouldRunDown;
	}

	//	@Override
//	// Broken atm, even more so now
//	public void simulationPeriodic() {
//		// Set inputs to the simulator
//		sim.setInputVoltage(currentVoltage);
//
//		// Update the sim (default time is 20 ms)
//		sim.update(DT50HZ);
//	}

	public void setGoal(double value) {
		goal = MathUtil.clamp(value, SHOOTER_ARM_RESTING_ANGLE, SHOOTER_ARM_MAX_ANGLE);
		if (goal > getEncoderPosition()) {
			PID.setPID(SA_P_UP, SA_I_UP, SA_D_UP);
			System.out.println("upgoalset");
		} else {
			PID.setPID(SA_P, SA_I, SA_D);
		}
	}

	public double getGoal() {
		return goal;
	}

	public boolean getUpperLimit() {
		return upperLimit.get();
	}

	public boolean getLowerLimit() {
		return lowerLimit.get();
	}

	/**
	 * Get the current position of the mechanism
	 * @return average encoder position in radians
	 */
	public double getEncoderPosition() {
		return encoderMotor.getSelectedSensorPosition() * 360 / 4096;
	}

	public void runPIDExecute() {
		double output = PID.calculate(getEncoderPosition(), goal);
		controllerOutput.setDouble(output);
		double cappedOutput = MathUtil.clamp(output, SHOOTER_ARM_DOWN_SPEED, SHOOTER_ARM_UP_SPEED);

		if (cappedOutput < 0 && !lowerLimit.get()) {
			set(cappedOutput);
		}

		if (cappedOutput > 0 && !upperLimit.get()) {
			set(cappedOutput);
		}
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

	public void set(double value) {
		       
        double clampedOutput = MathUtil.clamp(value, SHOOTER_ARM_DOWN_SPEED, SHOOTER_ARM_UP_SPEED);

		if ((value < 0 && !lowerLimit.get()) || (value > 0 && !upperLimit.get())) {
			leftMotor.set(clampedOutput);
			rightMotor.set(clampedOutput);
            System.out.println("CLAMPED OUTPUT: " + clampedOutput);
		} else {
			leftMotor.set(0);
			leftMotor.set(0);
		}
	}

    public void manualSet(double value) {
        double clampedOutput = MathUtil.clamp(value, SHOOTER_ARM_DOWN_SPEED, SHOOTER_ARM_UP_SPEED);

        leftMotor.set(clampedOutput);
        rightMotor.set(clampedOutput);
        System.out.println("CLAMPED OUTPUT: " + clampedOutput);
    }

	public boolean isAtTarget() {
		return Math.abs(getEncoderPosition() - goal) < SHOOTER_ARM_IS_FINISHED;
	}

	public void resetEncoder(double value) {
		encoderMotor.setSelectedSensorPosition(value / 360 * 4096);
	}

	public void resetEncoder() {
		encoderMotor.setSelectedSensorPosition(0);
	}

	public boolean isManualControl() {
		return manualControl;
	}

	public void setManualControl(boolean manualControl) {
		this.manualControl = manualControl;
	}
}
