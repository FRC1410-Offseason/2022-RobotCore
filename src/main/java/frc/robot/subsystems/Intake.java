package frc.robot.subsystems;

import com.revrobotics.*;
import frc.robot.framework.subsystem.SubsystemBase;

import static frc.robotmap.IDs.*;

public class Intake extends SubsystemBase {

	// Motor that runs the intake
	private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

	public Intake() {
		// Config motor
		intakeMotor.restoreFactoryDefaults();
	}

	/**
	 * Returns the speed of the intake
	 *
	 * @return Speed of motor
	 */
	public double getSpeed() {
		return intakeMotor.get();
	}

	/**
	 * Set the speed of the intake
	 *
	 * @param speed Speed from -1 to 1
	 */
	public void setSpeed(double speed) {
		intakeMotor.set(speed);
	}
}
