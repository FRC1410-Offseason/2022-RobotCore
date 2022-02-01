package frc.robot.subsystems;

import static frc.robotmap.Constants.*;

import frc.robot.framework.subsystem.SubsystemBase;

import com.revrobotics.*;

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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static frc.robotmap.IDs.*;

public class ShooterArm extends SubsystemBase {

    //<editor-fold desc="Hardware" defaultstate="collapsed">
    //Motors that run shooter arm
    private final CANSparkMax shooterArmLeftMotor = new CANSparkMax(SHOOTER_ARM_L_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax shooterArmRightMotor = new CANSparkMax(SHOOTER_ARM_R_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    //Encoder Objects for motors
    private final RelativeEncoder leftEncoder = shooterArmLeftMotor.getEncoder();
    private final RelativeEncoder rightEncoder = shooterArmRightMotor.getEncoder();

    //Locking Solenoid
    private final DoubleSolenoid brake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, SHOOTER_ARM_LOCK_FWD, SHOOTER_ARM_LOCK_BCK);


    //State variable to track state of lock
    private boolean lockState = false;
    //</editor-fold>

    //<editor-fold desc="State Space Stuff" default-state="collapsed">

    //State space model

    //LinearSystem<N2, N1, N1> armPlant = LinearSystemId.identifyPositionSystem(SHOOTER_ARM_KV, SHOOTER_ARM_KA);
    LinearSystem<N2, N1, N1> armPlant =
            LinearSystemId.createSingleJointedArmSystem(
                    DCMotor.getNEO(2),
                    SingleJointedArmSim.estimateMOI(SHOOTER_ARM_LENGTH, SHOOTER_ARM_MASS),
                    1
            );

    //Observer: Kalman Filter
    private final KalmanFilter<N2, N1, N1> armObserver =
            new KalmanFilter<>(
                    Nat.N2(),
                    Nat.N1(),
                    armPlant,
                    VecBuilder.fill(SHOOTER_ARM_POS_CONFIDENCE, SHOOTER_ARM_VEL_CONFIDENCE),
                    VecBuilder.fill(SHOOTER_ARM_ENC_CONFIDENCE),
                    DT
            );

    //Controller: Linear Quadratic Regulator
    private final LinearQuadraticRegulator<N2, N1, N1> armController =
            new LinearQuadraticRegulator<>(
              armPlant,
              VecBuilder.fill(SHOOTER_ARM_POS_TOLERANCE, SHOOTER_ARM_VEL_TOLERANCE),
              VecBuilder.fill(SHOOTER_ARM_CTRL_TOLERANCE),
              DT
            );

    private final TrapezoidProfile.Constraints armConstraints =
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(SHOOTER_ARM_MAX_VELOCITY),
                Units.degreesToRadians(SHOOTER_ARM_MAX_ACCEL)
            );

    private TrapezoidProfile.State lastProfiledReference = new TrapezoidProfile.State();
    private TrapezoidProfile.State currentTarget = new TrapezoidProfile.State();

    private final LinearSystemLoop<N2, N1, N1> armLoop =
            new LinearSystemLoop<>(armPlant, armController, armObserver, SHOOTER_ARM_MAX_VOLTAGE, DT);

    //</editor-fold>

    //<editor-fold desc="Sim Stuff" defaultstate="collapsed">
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getNEO(2),
            SHOOTER_ARM_GEARING,
            SingleJointedArmSim.estimateMOI(SHOOTER_ARM_LENGTH, SHOOTER_ARM_MASS),
            SHOOTER_ARM_LENGTH,
            Units.degreesToRadians(SHOOTER_ARM_MIN_ROT),
            Units.degreesToRadians(SHOOTER_ARM_MAX_ROT),
            SHOOTER_ARM_MASS,
            true,
            SHOOTER_ARM_NOISE
    );

    private final Mechanism2d mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d armPivot = mech2d.getRoot("Arm Pivot", 10, 10);
    private final MechanismLigament2d armTower = armPivot.append(new MechanismLigament2d("Arm Tower", 30, 0));

    private final MechanismLigament2d arm = armPivot.append(
            new MechanismLigament2d(
                    "Arm",
                    30,
                    Units.radiansToDegrees(armSim.getAngleRads()),
                    6,
                    new Color8Bit(Color.kYellow)
            )
    );

    private final Mechanism2d pistonMech = new Mechanism2d(60, 60);
    private final MechanismRoot2d brakeSim = pistonMech.getRoot("Piston", 30, 10);
    private final MechanismLigament2d pistonSim = brakeSim.append(new MechanismLigament2d("Piston Actuator", 20, 90));
    private final MechanismLigament2d pistonInnards = brakeSim.append(new MechanismLigament2d("Piston", 20, 90, 4, new Color8Bit(Color.kRed)));

    //</editor-fold>

    private double currentVoltage = 0;

    public ShooterArm() {
        //Reset motors to factory default
        shooterArmLeftMotor.restoreFactoryDefaults();
        shooterArmRightMotor.restoreFactoryDefaults();

        shooterArmLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterArmRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        SmartDashboard.putData("Arm Sim", mech2d);
        armTower.setColor(new Color8Bit(Color.kBlue));

        SmartDashboard.putData("Piston Sim", pistonMech);

        leftEncoder.setPositionConversionFactor((Math.PI * 2) / SHOOTER_ARM_GEARING);
        rightEncoder.setPositionConversionFactor((Math.PI * 2) / SHOOTER_ARM_GEARING);

        //Reset control loop to known state
        armLoop.reset(VecBuilder.fill(0, 0));
    }

    @Override
    public void periodic() {
        lastProfiledReference = (new TrapezoidProfile(armConstraints, currentTarget, lastProfiledReference)).calculate(DT);
        armLoop.setNextR(lastProfiledReference.position, lastProfiledReference.velocity);

        armLoop.correct(VecBuilder.fill(getPosition()));
        armLoop.predict(DT);
//        System.out.println(Math.abs(currentTarget.position - armLoop.getObserver().getXhat(0)));
        if (Math.abs(currentTarget.position - armLoop.getObserver().getXhat(0)) < 0.1) {
//        if (armLoop.getObserver().getXhat(1) < 0.001) {
            setBrake();
        } else {
            releaseBrake();
            setVoltage(armLoop.getU(0));
        }
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInputVoltage(currentVoltage);
        armSim.update(DT);
        if (getBrakeState()) {
            armSim.update(DT);
            pistonInnards.setLength(40);
        } else {
            pistonInnards.setLength(0);
        }
        leftEncoder.setPosition(armSim.getAngleRads());
        rightEncoder.setPosition(armSim.getAngleRads());
        arm.setAngle(Units.radiansToDegrees(armLoop.getObserver().getXhat(0)));
        System.out.println(Units.radiansToDegrees(armLoop.getObserver().getXhat(0)));
    }

    /**
     * Sets the reference of the controller
     * @param angle The angle of the arm in degrees
     */
    public void setTargetAngle(double angle) {
        System.out.println("Current target set to" + angle);
        this.currentTarget = new TrapezoidProfile.State(Units.degreesToRadians(angle), 0);
    }

    /**
     * Set bus voltage on the motors
     * @param voltage voltage (should probably 0 - 12)
     */
    public void setVoltage(double voltage) {
        currentVoltage = voltage;
        shooterArmLeftMotor.setVoltage(voltage);
        shooterArmRightMotor.setVoltage(voltage);
    }

    /**
     * Return the speed of the shooter arm
     * @return Speed of motors
     */
    public double getSpeed() {
        return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
    }

    /**
     * Get the average position across both encoders
     * @return position of the mechanism in radians
     */
    public double getPosition() { return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2; }

    /**
     * Set the state of the lock
     * @param state forward or backward
     */
    public void setBrake(Value state) {
        brake.set(state);
    }

    /**
     * Return the state of the lock
     * @return True / False -> Locked / Unlocked
     */
    public boolean getBrakeState() {
        return this.lockState;
    }

    /**
     * Set the state of the lock to locked
     */
    public void setBrake() {
        if (!this.lockState) {
            this.setBrake(Value.kForward);
            this.lockState = true;
        }
    }

    /**
     * Set the state of the lock to unlocked
     */
        public void releaseBrake() {
        if (this.lockState) {
            this.setBrake(Value.kReverse);
            this.lockState = false;
        }
    }

    /**
     * Toggle the state of the lock
     */
    public void toggle() {
        if (this.lockState) {
            this.releaseBrake();
        } else {
            this.setBrake();
        }
    }
}
