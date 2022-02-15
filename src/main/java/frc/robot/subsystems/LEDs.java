package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.framework.subsystem.SubsystemBase;
import org.intellij.lang.annotations.MagicConstant;

import static frc.robotmap.Constants.*;

public class LEDs extends SubsystemBase {

	private final PWMSparkMax controller = new PWMSparkMax(LED_CONTROLLER_PORT);

	private double profile = LEDProfile.OFF;

	public interface LEDProfile {
		double OFF = 0.99;
		double GREEN = 0.75;
		double BLUE = 0.85;
		double RED = 0.61;
	}

	@Override
	public void periodic() {
		controller.set(profile);
	}

	/**
	 * Returns the current profile
	 * @return double from -0.99 to 0.99
	 */
	public double getProfile() {
		return profile;
	}

	/**
	 * Raw input for controller
	 * @param desiredProfile -0.99 -> 0.99
	 */
	public void setRawProfile(double desiredProfile) {
		if (desiredProfile != profile) {
			profile = desiredProfile;
		}
	}

	/**
	 * Magic
	 * @param desiredProfile magic
	 */
	public void setProfile(@MagicConstant(valuesFromClass = LEDProfile.class) double desiredProfile) {
		profile = desiredProfile;
	}
}

