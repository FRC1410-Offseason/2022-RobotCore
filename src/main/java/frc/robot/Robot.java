package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.commands.actions.SetShooterRPM;
import frc.robot.commands.actions.ToggleShooterArmPosition;
import frc.robot.commands.grouped.*;
import frc.robot.commands.looped.*;
import frc.robot.commands.actions.*;
import frc.robot.framework.scheduler.RobotMode;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.subsystems.*;
import frc.robot.util.Trajectories;

import static frc.robotmap.Constants.*;
import static frc.robotmap.IDs.PRESSURE_SENSOR;

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
	private final Elevator elevator = new Elevator();
	private final Intake intake = new Intake();
	private final IntakeFlipper intakeFlipper = new IntakeFlipper();
	private final Shooter shooter = new Shooter();
	private final Storage storage = new Storage(DriverStation.getAlliance());
	private final ShooterArm shooterArm = new ShooterArm(storage.getShooterArmMotor());
	private final Winch winch = new Winch();
	private final Limelight limelight = new Limelight();
	private final Trajectories auto = new Trajectories(drivetrain);

	@Override
	public void registerControls() {
		getOperatorRightBumper().whileHeld(new ToggleIntake(intakeFlipper));
		// Toggle intake position
//		getOperatorRightBumper().whenPressed(new ToggleIntake(intakeFlipper)); //TODO: Reenable after intake flipper is ready

		// Toggle shooter arm position
		getOperatorLeftBumper().whenPressed(new ToggleShooterArmPosition(shooterArm));

		// Set storage speed
		getOperatorYButton().whileHeld(new RunStorageConstant(storage, STORAGE_RUN_SPEED));

//		getDriverAButton().whenPressed(new RunCommand(() -> winch.lock(), winch));
//		getDriverXButton().whenPressed(new RunCommand(() -> winch.unlock(), winch));
		// Set shooter rpm
		getOperatorXButton().whenPressed(new SetShooterRPM(shooter, NetworkTables.getShooterHighRPM()));

		// Limelight align to target and shoot
		getDriverRightBumper().whenPressed(new LimelightShoot(drivetrain, limelight, shooter, shooterArm, storage, NetworkTables.getShooterHighRPM()));

		//
		getDriverLeftBumper().whenPressed(new LowHubShoot(shooter, shooterArm, storage, NetworkTables.getShooterLowRPM()));

		// Climb cycle dpad control
		getOperatorDPadUp().whileHeld(new RunElevatorConstant(elevator, ELEVATOR_UP_SPEED));
		getOperatorDPadRight().whileHeld(new RunWinchConstant(winch, WINCH_OUT_SPEED));
		getOperatorDPadDown().whileHeld(new RunElevatorConstant(elevator, ELEVATOR_DOWN_SPEED));
		getOperatorDPadLeft().whileHeld(new RunWinchConstant(winch, WINCH_IN_SPEED));
	}

	@Override
	public void robotInit() {
		NetworkTables.setAutoList(autoList);
		NetworkTables.setCorrectColor(DriverStation.getAlliance().toString());
		NetworkTables.setPressure(pressure);
		drivetrain.setCoast();

		resetAngle.setDouble(0);

		shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);
	}

	@Override
	public void autonomousInit() {
		scheduler.scheduleDefaultCommand(new PoseEstimation(drivetrain), TIME_OFFSET, 10);
		shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);
		scheduler.scheduleDefaultCommand(new RunShooterArm(shooterArm));
		drivetrain.setBrake();



		switch ((int) NetworkTables.getAutoChooser()) {
			case 0:
				break;
			case 1:
				break;
			case 2:
				break;
			case 3:
				autonomousCommand = new TwoCargoAuto(auto, drivetrain, intake, storage, shooterArm, shooter, intakeFlipper, limelight, NetworkTables.getAutoRPM());
				break;
			default: throw new IllegalStateException("Unknown auto profile " + auto);
		}


		scheduler.scheduleDefaultCommand(autonomousCommand, TIME_OFFSET, 10, RobotMode.TELEOP, RobotMode.TEST);
		scheduler.debugDumpList();
	}

	@Override
	public void teleopInit() {
		shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);
		drivetrain.setBrake();

		// Tank drive on the drivetrain
		 scheduler.scheduleDefaultCommand(new TankDrive(drivetrain, getDriverLeftYAxis(), getDriverRightYAxis()));

		// Telescoping arms on the operator controller
		scheduler.scheduleDefaultCommand(new RunElevator(elevator, getOperatorLeftYAxis()));

		// Run the intake (and storage) on the operator right trigger
		scheduler.scheduleDefaultCommand(new RunIntake(intake, storage, getOperatorRightTrigger()));
		scheduler.scheduleDefaultCommand(new RunIntakeFlipperWithAxis(intakeFlipper, getOperatorLeftYAxis()));

		// Run the intake flipper
//		scheduler.scheduleDefaultCommand(new RunIntakeFlipper(intakeFlipper)); //TODO: Reenable after intake flipper is ready

		// Run the shooter arm
//		scheduler.scheduleDefaultCommand(new RunShooterArm(shooterArm));
//		scheduler.scheduleDefaultCommand(new RunArmWithAxis(shooterArm, getOperatorLeftYAxis()));

		// Run the storage
		scheduler.scheduleDefaultCommand(new RunStorage(storage));

		// Run the winches on the operator controller
//		scheduler.scheduleDefaultCommand(new RunWinch(winch, getOperatorRightYAxis()));
		drivetrain.setBrake();
	}

	@Override
	public void testInit() {
		scheduler.scheduleDefaultCommand(new RunArmWithAxis(shooterArm, getOperatorLeftYAxis()));
		scheduler.scheduleDefaultCommand(new RunIntakeFlipperWithAxis(intakeFlipper, getOperatorRightYAxis()));

		getOperatorXButton().whenPressed(new ResetShooterArmEncoderWithEntry(shooterArm, resetAngle));

		getOperatorYButton().whenPressed(new LockWinches(winch));

		autonomousCommand.cancel();
		drivetrain.setCoast();
	}
}
