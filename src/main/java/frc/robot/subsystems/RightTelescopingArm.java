package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.framework.subsystem.SubsystemBase;

import static frc.robotmap.Constants.*;
import static frc.robotmap.IDs.*;

public class RightTelescopingArm extends SubsystemBase {

	// Elevator motor
	private final CANSparkMax rightMotor = new CANSparkMax(ELEVATOR_RIGHT_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
	private double currentVoltage = 0;

	/**
	 * Grab the encoders from the motor object
	 */
	private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

	// Elevator Brake
//	private final DoubleSolenoid lock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ELEVATOR_FWD, ELEVATOR_BCK);

	/**
	 * The simulation system
	 */
	private final LinearSystemSim<N2, N1, N1> sim = new LinearSystemSim<>(
				LinearSystemId.createElevatorSystem(
				DCMotor.getNEO(1),
				ELEVATOR_MASS,
				Units.inchesToMeters(1),
				GEAR_RATIO
		)
	);

	/**
	 * Used for the widget that shows the height of hte elevator in the simulator
	 */
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

	public RightTelescopingArm() {
		//Reset the motors
		rightMotor.restoreFactoryDefaults();
		
		//The motors to brake mode
		rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		//Set the conversion factor for the encoders so that they report in meters instead of rev ticks
		rightEncoder.setPositionConversionFactor(ELEVATOR_METERS_PER_REV);

		// Send the sim stuff to the simulation over networktables
		SmartDashboard.putData("Right Elevator Sim", simWidget);
		SmartDashboard.putData("Right Elevator Piston", pistonSim);
	}

	@Override
	public void simulationPeriodic() {
		sim.setInput(getSpeed() * RobotController.getBatteryVoltage());

//		if (lock.get() == Value.kForward) {
//			pistonInnards.setLength(40);
//			sim.update(0);
//		} else {
//			pistonInnards.setLength(0);
//			sim.update(DT50HZ);
//		}

		rightEncoder.setPosition(sim.getOutput(0));

		elevatorSim.setLength(sim.getOutput(0) * 23);
	}

	/**
	 * Set the voltage of the motors
	 * @param voltage -12 to 12 volts
	 */
	public void setVoltage(double voltage) {
		currentVoltage = voltage;
		rightMotor.setVoltage(voltage);
	}

	/**
	 * Set the speed of the motors
	 * @param speed a double -1 to 1
	 */
	public void set(double speed) {
		rightMotor.set(speed);
	}

	/**
	 * Get the current speed as a double between -1 and 1
	 * @return a double between -1 and 1
	 */
	public double getSpeed() {
		return rightMotor.get();
	}

	/**
	 * Get the current voltage applied to the motors
	 * @return a voltage between -12 and 12
	 */
	public double getCurrentVoltage() {
		return currentVoltage;
	}
}
