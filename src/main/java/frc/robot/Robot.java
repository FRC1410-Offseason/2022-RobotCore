package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.actions.SetShooterRPM;
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
//		getOperatorRightBumper().whileHeld(new ToggleIntake(intakeFlipper));
		// Toggle intake position
//		getOperatorRightBumper().whenPressed(new ToggleIntake(intakeFlipper)); //TODO: Reenable after intake flipper is ready

		// Set storage speed
		getOperatorYButton().whileHeld(new RunStorageConstant(storage, STORAGE_RUN_SPEED));

		// Set shooter rpm
		getOperatorXButton().whenPressed(new SetShooterRPM(shooter, NetworkTables.getShooterHighRPM()));

		// Limelight align to target and shoot
//		getDriverRightBumper().whenPressed(new LimelightShoot(drivetrain, limelight, shooter, shooterArm, storage, NetworkTables.getShooterHighRPM()));

		//
//		getDriverLeftBumper().whenPressed(new LowHubShoot(shooter, shooterArm, storage, NetworkTables.getShooterLowRPM()));

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

		resetAngle.setDouble(SHOOTER_ARM_MAX_ANGLE);

//		shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);
	}

	@Override
	public void autonomousInit() {
		scheduler.scheduleDefaultCommand(new PoseEstimation(drivetrain), TIME_OFFSET, 10);
		scheduler.scheduleDefaultCommand(new RunIntakeFlipper(intakeFlipper));
		scheduler.scheduleDefaultCommand(new RunShooterArm(shooterArm));
		shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);
		drivetrain.setBrake();
		intakeFlipper.resetEncoders(0);

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
		scheduler.scheduleDefaultCommand(new RunShooterArm(shooterArm));
		drivetrain.setBrake();

		getOperatorRightBumper().whenPressed(new InstantCommand(() -> shooterArm.setGoal(SHOOTER_ARM_MAX_ANGLE)));
		getOperatorLeftBumper().whenPressed(new InstantCommand(() -> shooterArm.setGoal(SHOOTER_ARM_INTAKE_ANGLE)));


		// Tank drive on the drivetrain
		// Yes, the right left is swapped, it's supposed to be that way trust me
		 scheduler.scheduleDefaultCommand(new TankDrive(drivetrain, getDriverLeftYAxis(), getDriverRightYAxis()));

		// Telescoping arms on the operator controller
//		scheduler.scheduleDefaultCommand(new RunElevator(elevator, getOperatorLeftYAxis()));

		// Run the intake (and storage) on the operator right trigger
		scheduler.scheduleDefaultCommand(new RunIntake(intake, storage, getOperatorRightTrigger()));
		scheduler.scheduleDefaultCommand(new RunIntakeFlipperWithAxis(intakeFlipper, getOperatorLeftYAxis()));

		// Run the intake flipper
//		scheduler.scheduleDefaultCommand(new RunIntakeFlipper(intakeFlipper));

		// Run the storage
		scheduler.scheduleDefaultCommand(new RunStorage(storage));

		// Run the winches on the operator controller
//		scheduler.scheduleDefaultCommand(new RunWinch(winch, getOperatorRightYAxis()));

		drivetrain.setBrake();
	}

	@Override
	public void testInit() {
		intakeFlipper.resetEncoders(0);
		shooterArm.resetEncoder(SHOOTER_ARM_INTAKE_ANGLE);
		scheduler.scheduleDefaultCommand(new RunArmWithAxis(shooterArm, getOperatorLeftYAxis()));
		scheduler.scheduleDefaultCommand(new RunIntakeFlipperWithAxis(intakeFlipper, getOperatorRightYAxis()));
//		scheduler.scheduleDefaultCommand(new RunShooterArm(shooterArm));
//		scheduler.scheduleDefaultCommand(new RunIntakeFlipper(intakeFlipper));
//		getOperatorXButton().whenPressed(new ResetShooterArmEncoderWithEntry(shooterArm, resetAngle));
//
//		getOperatorAButton().whenPressed(new LockElevator(elevator));
//		getOperatorRightBumper().whenPressed(new RaiseShooterArm(shooterArm));
//		getOperatorLeftBumper().whenPressed(new LowerShooterArm(shooterArm));
		getOperatorXButton().whenPressed(new InstantCommand(() -> intakeFlipper.resetEncoders(INTAKE_DOWN_POSITION)));


//		getOperatorLeftBumper().whenPressed(new InstantCommand(() -> shooterArm.setGoal(SHOOTER_ARM_INTAKE_ANGLE)));
//		getOperatorRightBumper().whenPressed(new InstantCommand(() -> shooterArm.setGoal(SHOOTER_ARM_MAX_ANGLE)));

//		getOperatorLeftBumper().whenPressed(new RetractIntake(intakeFlipper));

//		getOperatorRightBumper().whenPressed(new ExtendIntake(intakeFlipper));

		getOperatorDPadUp().whenPressed(new SetShooterRPM(shooter, NetworkTables.getShooterHighRPM()));
		getOperatorDPadDown().whenPressed(new SetShooterRPM(shooter, 0));

		getOperatorAButton().whileHeld(new RunStorageConstant(storage, STORAGE_RUN_SPEED));

//		getOperatorYButton().whenPressed(new LockWinches(winch));

		drivetrain.setCoast();
	}

	@Override
	public void testPeriodic() {
		System.out.println(intakeFlipper.getEncoderPosition());
	}
}
