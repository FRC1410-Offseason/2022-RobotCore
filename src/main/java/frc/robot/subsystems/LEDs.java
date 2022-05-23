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
		double PINK = 0.57;
		double TWOCOLORSWITCH = -0.61;
        double RAINBOW = 0.99; // -0.97
	}

	@Override
	public void periodic() {
		controller.set(profile);
	}

	/**
	 * Magic
	 * @param desiredProfile magic
	 */
	public void setProfile(@MagicConstant(valuesFromClass = LEDProfile.class) double desiredProfile) {
		profile = desiredProfile;
	}
}

