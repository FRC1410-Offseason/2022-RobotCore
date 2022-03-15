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
import edu.wpi.first.wpilibj.DoubleSolenoid;
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

	public boolean isArmUp() {
		return isArmUp;
	}

	public void extend() {
		armSolenoid.set(DoubleSolenoid.Value.kForward);
		isArmUp = true;
	}

	public void retract() {
		armSolenoid.set(DoubleSolenoid.Value.kReverse);
		isArmUp = false;
	}

	public void toggle() {
		if (isArmUp) {
			retract();
		} else {
			extend();
		}
	}
}
