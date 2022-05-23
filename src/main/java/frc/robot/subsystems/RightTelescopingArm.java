package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.framework.subsystem.SubsystemBase;

import static frc.robotmap.Constants.*;
import static frc.robotmap.IDs.*;

public class RightTelescopingArm extends SubsystemBase {

	// Elevator motor
	private final CANSparkMax rightMotor = new CANSparkMax(ELEVATOR_RIGHT_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

	/**
	 * Grab the encoders from the motor object
	 */
	private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

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

	public RightTelescopingArm() {
		//Reset the motors
		rightMotor.restoreFactoryDefaults();
		
		//The motors to brake mode
		rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		//Set the conversion factor for the encoders so that they report in meters instead of rev ticks
		rightEncoder.setPositionConversionFactor(ELEVATOR_METERS_PER_REV);

		// Send the sim stuff to the simulation over networktables
		SmartDashboard.putData("Right Elevator Sim", simWidget);
	}

	@Override
	public void simulationPeriodic() {
		sim.setInput(getSpeed() * RobotController.getBatteryVoltage());

		rightEncoder.setPosition(sim.getOutput(0));

		elevatorSim.setLength(sim.getOutput(0) * 23);
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
}
