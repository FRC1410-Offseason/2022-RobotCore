package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.framework.subsystem.SubsystemBase;

import static frc.robotmap.IDs.*;

public class ShooterArm extends SubsystemBase {

	private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
	private final NetworkTable table = instance.getTable("Shooter Arm");
	private final NetworkTableEntry armPosNT = table.getEntry("Shooter Arm Position");

	private final DoubleSolenoid armSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SHOOTER_ARM_SOLENOID_FWD, SHOOTER_ARM_SOLENOID_BCK);

	private boolean isArmUp = true;

	public ShooterArm() {
		armPosNT.setString("Up");
	}

	@Override
	public void periodic() {
		if (isArmUp) {
			armPosNT.setString("Up");
		} else {
			armPosNT.setString("Down");
		}
	}

	public void extend() {
		armSolenoid.set(DoubleSolenoid.Value.kReverse);
		isArmUp = true;
	}

	public void retract() {
		armSolenoid.set(DoubleSolenoid.Value.kForward);
		isArmUp = false;
	}
}
