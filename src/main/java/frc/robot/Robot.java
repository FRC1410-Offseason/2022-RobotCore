package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.commands.actions.SetShooterRPM;
import frc.robot.commands.actions.ToggleShooterArmPosition;
import frc.robot.commands.grouped.*;
import frc.robot.commands.looped.*;
import frc.robot.framework.scheduler.RobotMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.looped.*;
import frc.robot.commands.actions.*;
import frc.robot.commands.grouped.*;
import frc.robot.framework.scheduler.EnqueuedTask;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.framework.scheduler.TaskScheduler;
import frc.robot.subsystems.*;
import frc.robot.util.Trajectories;

import static frc.robotmap.Constants.*;
import static frc.robotmap.IDs.PRESSURE_SENSOR;

public class Robot extends ScheduledRobot {

	private final String[] autoList = {"0 - Taxi", "1 - 2CargoDrive", "2 - 2CargoNoSA", "3 - 2CargoAuto"};
	private final AnalogInput pressure = new AnalogInput(PRESSURE_SENSOR);
	CommandGroupBase autonomousCommand = null;

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
		// Toggle intake position
//		getOperatorRightBumper().whenPressed(new ToggleIntake(intakeFlipper)); //TODO: Reenable after intake flipper is ready

		// Toggle shooter arm position
		getOperatorLeftBumper().whenPressed(new ToggleShooterArmPosition(shooterArm));

		// Set storage speed
		getOperatorYButton().whileHeld(new RunStorageConstant(storage, STORAGE_RUN_SPEED));

		// Set shooter rpm
		getOperatorXButton().whenPressed(new SetShooterRPM(shooter, NetworkTables.getShooterTargetRPM()));

		// Limelight align to target and shoot
		getDriverRightBumper().whileHeld(new LimelightShoot(drivetrain, limelight, shooter, storage, 2055));

		// Climb cycle dpad control
		getOperatorDPadUp().whileHeld(new RunElevatorConstant(elevator, ELEVATOR_UP_SPEED));
		getOperatorDPadRight().whileHeld(new RunWinchConstant(winch, WINCH_OUT_SPEED));
		getOperatorDPadDown().whileHeld(new RunElevatorConstant(elevator, ELEVATOR_DOWN_SPEED));
		getOperatorDPadLeft().whileHeld(new RunWinchConstant(winch, WINCH_IN_SPEED));
    
		// All teleop
		getDriverLeftBumper().whenPressed(new LimelightAnglePID(limelight, drivetrain));
	}

	@Override
	public void robotInit() {
		NetworkTables.setAutoList(autoList);
		NetworkTables.setCorrectColor(DriverStation.getAlliance().toString());
		NetworkTables.setPressure(pressure);
		drivetrain.setCoast();

		shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);
	}

	@Override
	public void autonomousInit() {
		scheduler.scheduleDefaultCommand(new PoseEstimation(drivetrain), TIME_OFFSET, 10);
		shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);
		scheduler.scheduleDefaultCommand(new RunShooterArm(shooterArm));
		// scheduler.scheduleDefaultCommand(new RunStorage(storage));
		drivetrain.setBrake();
	
		

		switch ((int) NetworkTables.getAutoChooser()) {
			case 0:
				// autonomousCommand = new TaxiAuto(auto, drivetrain);
				break;
			case 1:
				// autonomousCommand = new TwoCargoAutoDrive(auto, drivetrain, limelight);
				break;
			case 2:
				// autonomousCommand = new TwoCargoAutoNoSA(auto, drivetrain, intake, storage, shooter, intakeFlipper, limelight, NetworkTables.getAutoRPM());
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
		autonomousCommand.cancel();
		drivetrain.setBrake();

		// Tank drive on the drivetrain
		// scheduler.scheduleDefaultCommand(new TankDrive(drivetrain, getDriverLeftYAxis(), getDriverRightYAxis()));

		// Telescoping arms on the operator controller
		scheduler.scheduleDefaultCommand(new RunElevator(elevator, getOperatorLeftYAxis()));

		// Run the intake (and storage) on the operator right trigger
		scheduler.scheduleDefaultCommand(new RunIntake(intake, storage, getOperatorRightTrigger()));

		// Run the intake flipper
//		scheduler.scheduleDefaultCommand(new RunIntakeFlipper(intakeFlipper)); //TODO: Reenable after intake flipper is ready

		// Run the shooter arm
		scheduler.scheduleDefaultCommand(new RunShooterArm(shooterArm));

		// Run the storage
		scheduler.scheduleDefaultCommand(new RunStorage(storage));

		// Run the winches on the operator controller
		scheduler.scheduleDefaultCommand(new RunWinch(winch, getOperatorRightYAxis()));
		drivetrain.setBrake();
	}

	@Override
	public void testInit() {
		autonomousCommand.cancel();
		drivetrain.setCoast();
	}
}
