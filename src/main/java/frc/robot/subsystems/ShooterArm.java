package frc.robot.subsystems;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.framework.subsystem.SubsystemBase;

public class ShooterArm extends SubsystemBase {

    private final CANSparkMax leftMotor = new CANSparkMax(SHOOTER_ARM_L_MOTOR, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(SHOOTER_ARM_R_MOTOR, MotorType.kBrushless);

    private double currentVoltage = 0;

    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

    private final DoubleSolenoid brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SHOOTER_ARM_LOCK_FWD, SHOOTER_ARM_LOCK_BCK);

    private final LinearSystem<N2, N1, N1> plant =
            LinearSystemId.createSingleJointedArmSystem(
                    DCMotor.getNEO(2),
                    SingleJointedArmSim.estimateMOI(SHOOTER_ARM_LENGTH, SHOOTER_ARM_MASS),
                    125
            );

    private final KalmanFilter<N2, N1, N1> observer =
            new KalmanFilter<>(
                    Nat.N2(),
                    Nat.N1(),
                    plant,
                    VecBuilder.fill(SHOOTER_ARM_POS_CONFIDENCE, SHOOTER_ARM_VEL_CONFIDENCE),
                    VecBuilder.fill(SHOOTER_ARM_ENC_CONFIDENCE),
                    DT
            );

    private final LinearQuadraticRegulator<N2, N1, N1> controller =
            new LinearQuadraticRegulator<>(
                    plant,
                    VecBuilder.fill(SHOOTER_ARM_POS_TOLERANCE, SHOOTER_ARM_VEL_TOLERANCE),
                    VecBuilder.fill(SHOOTER_ARM_CTRL_TOLERANCE),
                    DT
            );

    private final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(SHOOTER_ARM_MAX_VELOCITY),
                    Units.degreesToRadians(SHOOTER_ARM_MAX_ACCEL)
            );

    private final LinearSystemLoop<N2, N1, N1> loop =
            new LinearSystemLoop<>(plant, controller, observer, SHOOTER_ARM_MAX_VOLTAGE, DT);

    private final REVPhysicsSim encoderSim = new REVPhysicsSim();

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

    public ShooterArm() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftEncoder.setPositionConversionFactor(Math.PI * 2);
        rightEncoder.setPositionConversionFactor(Math.PI * 2);
        leftEncoder.setVelocityConversionFactor(Math.PI * 2);
        rightEncoder.setVelocityConversionFactor(Math.PI * 2);

        SmartDashboard.putData("Arm Sim", simMech);
        tower.setColor(new Color8Bit(Color.kBlue));

        SmartDashboard.putData("Piston Sim", pistonSim);

        setBrake();

        encoderSim.addSparkMax(leftMotor, DCMotor.getNEO(1));
        encoderSim.addSparkMax(rightMotor, DCMotor.getNEO(1));

        loop.reset(VecBuilder.fill(getEncoderPosition(), getEncoderVelocity()));
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {
        if (getBrakeState()) {
            pistonInnards.setLength(40);
        } else {
            pistonInnards.setLength(0);
        }
        sim.setInputVoltage(this.currentVoltage);
        sim.update(DT);
        leftEncoder.setPosition(sim.getAngleRads());
        rightEncoder.setPosition(sim.getAngleRads());
        arm.setAngle(Units.radiansToDegrees(loop.getObserver().getXhat(0)));

    }

    public double getEncoderPosition() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    }

    public double getEncoderVelocity() {
        return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
    }

    public LinearSystemLoop<N2, N1, N1> getLoop() {
        return loop;
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return constraints;
    }

    public void setVoltage(double voltage) {
        this.currentVoltage = voltage;
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    public boolean getBrakeState() {
        return brake.get() == Value.kForward;
    }

    public void setBrake() {
        brake.set(Value.kForward);
    }

    public void releaseBrake() {
        brake.set(Value.kReverse);
    }
}

