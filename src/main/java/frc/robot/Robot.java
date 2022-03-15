package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.grouped.*;
import frc.robot.commands.looped.*;
import frc.robot.commands.actions.*;
import frc.robot.framework.scheduler.RobotMode;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.subsystems.*;
import frc.robot.util.Trajectories;

import static frc.robotmap.Constants.*;
import static frc.robotmap.IDs.PRESSURE_SENSOR;
import static frc.robotmap.Tuning.*;

public class Robot extends ScheduledRobot {

	private final String[] autoList = {"0 - Taxi", "1 - 2CargoDrive", "2 - 2CargoNoSA", "3 - 2CargoAuto"};
	private final AnalogInput pressure = new AnalogInput(PRESSURE_SENSOR);
	CommandGroupBase autonomousCommand = null;

	private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
	private final NetworkTable table = instance.getTable("Shooter Arm");
	private final NetworkTableEntry resetAngle = table.getEntry("Reset arm to: ");

	public static void main(String[] args) {RobotBase.startRobot(Robot::new);}
	private Robot() {
		super((long) DT50HZ);
	}

	private final Drivetrain drivetrain = new Drivetrain();
    private final LeftTelescopingArm leftTA = new LeftTelescopingArm();
    private final RightTelescopingArm rightTA = new RightTelescopingArm();
	private final Intake intake = new Intake();
	private final IntakeFlipper intakeFlipper = new IntakeFlipper();
	private final Shooter shooter = new Shooter();
	private final Storage storage = new Storage(DriverStation.getAlliance());
	private final ShooterArm shooterArm = new ShooterArm();
	private final Winch winch = new Winch();
	// private final Limelight limelight = new Limelight();
	private final Trajectories auto = new Trajectories(drivetrain);

	@Override
	public void robotInit() {
		NetworkTables.setAutoList(autoList);
		NetworkTables.setCorrectColor(DriverStation.getAlliance().toString());
		NetworkTables.setPressure(pressure);

		resetAngle.setDouble(SHOOTER_ARM_MAX_ANGLE);

//		shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);
	}

	@Override
	public void autonomousInit() {
        System.out.println("INITIALIZING AUTO");
		scheduler.scheduleDefaultCommand(new PoseEstimation(drivetrain), TIME_OFFSET, 10);
		scheduler.scheduleDefaultCommand(new RunIntakeFlipper(intakeFlipper));
		drivetrain.setBrake();
		intakeFlipper.resetEncoders(0);

		switch ((int) NetworkTables.getAutoChooser()) {
			case 0:
                autonomousCommand = new SequentialCommandGroup();
				break;
			case 1:
				autonomousCommand = new TaxiAuto(auto, drivetrain);
				break;
			case 2:
				autonomousCommand = new OneCargoLowTime(auto, drivetrain, intake, storage, shooterArm, shooter, intakeFlipper, SHOOTER_LOW_HUB_RPM);
				break;
			case 3:
				autonomousCommand = new TwoCargoLow(auto, drivetrain, intake, storage, shooterArm, shooter, intakeFlipper, SHOOTER_LOW_HUB_RPM);
				break;
            case 4:
                autonomousCommand = new TwoCargoLow(auto, drivetrain, intake, storage, shooterArm, shooter, intakeFlipper, SHOOTER_LOW_HUB_RPM);
                break;
			default: throw new IllegalStateException("Unknown auto profile " + auto);
		}


		scheduler.scheduleDefaultCommand(autonomousCommand, TIME_OFFSET, 10, RobotMode.TELEOP, RobotMode.TEST);
		scheduler.debugDumpList();
	}

    @Override
	public void registerControls() {
		// Set storage speed
		getOperatorYButton().whileHeld(new RunIntakeWithButton(intake, storage, shooter));
		getOperatorXButton().whileHeld(new RunIntakeWithButton(intake, storage, shooter));

        getOperatorRightBumper().whenPressed(new RaiseShooterArm(shooterArm));
        getOperatorRightBumper().whenPressed(new RetractIntake(intakeFlipper));
		getOperatorLeftBumper().whenPressed(new LowerShooterArm(shooterArm));
        getOperatorLeftBumper().whenPressed(new ExtendIntakeDelayed(intakeFlipper));

		getOperatorDPadLeft().whileHeld(new LowerShooterArm(shooterArm));
		getOperatorDPadRight().whileHeld(new RaiseShooterArm(shooterArm));
        getOperatorDPadRight().whenPressed(new RetractIntake(intakeFlipper));

		getOperatorDPadUp().whenPressed(new SetShooterRPM(shooter, SHOOTER_LOW_HUB_RPM));
		getOperatorDPadDown().whenPressed(new SetShooterRPM(shooter, 0));

		// Low Hub Shoot
		getDriverAButton().whenPressed(new LowHubShoot(shooter, shooterArm, storage, SHOOTER_LOW_HUB_RPM));

        getDriverYButton().whenPressed(new FlipDrivetrain(drivetrain));

        getDriverLeftBumper().whileHeld(new RunLeftTAConstant(leftTA, TA_RAISE_SPEED));
        getDriverRightBumper().whileHeld(new RunRightTAConstant(rightTA, TA_RAISE_SPEED));
	}

	@Override
	public void teleopInit() {
		scheduler.scheduleDefaultCommand(new RunIntakeFlipper(intakeFlipper));
//		scheduler.scheduleDefaultCommand(new RunShooterArm(shooterArm));
		drivetrain.setBrake();

		// Toggle the shooter arm
//		getOperatorRightBumper().whenPressed(new InstantCommand(() -> shooterArm.setGoal(SHOOTER_ARM_MAX_ANGLE)));
//		getOperatorLeftBumper().whenPressed(new InstantCommand(() -> shooterArm.setGoal(SHOOTER_ARM_INTAKE_ANGLE)));

		// Tank drive on the drivetrain
		scheduler.scheduleDefaultCommand(new TankDrive(drivetrain, getDriverLeftYAxis(), getDriverRightYAxis()));

		// Telescoping arms on the operator controller
		scheduler.scheduleDefaultCommand(new RunLeftTA(leftTA, getDriverLeftTrigger()));
        scheduler.scheduleDefaultCommand(new RunRightTA(rightTA, getDriverRightTrigger()));

		// Run the storage
		scheduler.scheduleDefaultCommand(new RunStorage(storage));

		// Run the winches on the operator controller
		scheduler.scheduleDefaultCommand(new RunWinch(winch, getOperatorLeftYAxis(), getOperatorRightYAxis()));

		drivetrain.setBrake();
	}

	@Override
	public void testInit() {
		drivetrain.setCoast();

		intakeFlipper.resetEncoders(0);
//		shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);
		scheduler.scheduleDefaultCommand(new RunIntakeFlipperWithAxis(intakeFlipper, getOperatorRightYAxis()));
//		scheduler.scheduleDefaultCommand(new RunShooterArm(shooterArm));
//		scheduler.scheduleDefaultCommand(new RunIntakeFlipper(intakeFlipper));
//		getOperatorXButton().whenPressed(new ResetShooterArmEncoderWithEntry(shooterArm, resetAngle));
//
//		getOperatorAButton().whenPressed(new LockElevator(elevator));
//		getOperatorRightBumper().whenPressed(new RaiseShooterArm(shooterArm));
//		getOperatorLeftBumper().whenPressed(new LowerShooterArm(shooterArm));
		getOperatorXButton().whenPressed(new InstantCommand(() -> intakeFlipper.resetEncoders(INTAKE_UP_POSITION)));
		getOperatorYButton().whenPressed(new InstantCommand(() -> intakeFlipper.resetEncoders(INTAKE_DOWN_POSITION)));

//		getOperatorLeftBumper().whenPressed(new InstantCommand(() -> shooterArm.setGoal(SHOOTER_ARM_INTAKE_ANGLE)));
//		getOperatorRightBumper().whenPressed(new InstantCommand(() -> shooterArm.setGoal(SHOOTER_ARM_MAX_ANGLE)));

//		getOperatorLeftBumper().whenPressed(new RetractIntake(intakeFlipper));

//		getOperatorRightBumper().whenPressed(new ExtendIntake(intakeFlipper));

		getOperatorDPadUp().whenPressed(new SetShooterRPM(shooter, SHOOTER_LOW_HUB_RPM));
		getOperatorDPadDown().whenPressed(new SetShooterRPM(shooter, 0));

		getOperatorAButton().whileHeld(new RunStorageConstant(storage, STORAGE_RUN_SPEED));

//		getOperatorYButton().whenPressed(new LockWinches(winch));
	}

	@Override
	public void testPeriodic() {
		System.out.println(intakeFlipper.getEncoderPosition());
	}
}
