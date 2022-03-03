package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.looped.*;
import frc.robot.commands.actions.*;
import frc.robot.commands.grouped.*;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.framework.scheduler.TaskScheduler;
import frc.robot.framework.scheduler.task.CommandTask;
import frc.robot.subsystems.*;
import frc.robot.util.Trajectories;

import static frc.robotmap.Constants.*;

import static frc.robotmap.IDs.PRESSURE_SENSOR;

public class Robot extends ScheduledRobot {

	private final String[] autoList = {"Taxi", "2Cargo", "3CargoTerminal", "3CargoUpRight", "4Cargo", "5Cargo"};
	private final AnalogInput pressure = new AnalogInput(PRESSURE_SENSOR);
	private CommandTask autoTask = null;
	private Command autonomousCommand = null;

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
	public TaskScheduler getScheduler() {
		return scheduler;
	}

	@Override
	public void registerControls() {
		//<editor-fold desc="Defaults">
		// Tank drive on the drivetrain
		scheduler.scheduleDefaultCommand(new TankDrive(drivetrain, getDriverLeftYAxis(), getDriverRightYAxis()));

		// Telescoping arms on the operator controller
		scheduler.scheduleDefaultCommand(new RunElevator(elevator, getOperatorLeftYAxis()));

		// Run the intake (and storage) on the operator right trigger
		scheduler.scheduleDefaultCommand(new RunIntake(intake, storage, getOperatorRightTrigger()));

		// Run the intake flipper
		scheduler.scheduleDefaultCommand(new RunIntakeFlipper(intakeFlipper));

		// Run the shooter arm
		scheduler.scheduleDefaultCommand(new RunShooterArm(shooterArm));

		// Run the winches on the operator controller
		scheduler.scheduleDefaultCommand(new RunWinch(winch, getOperatorRightYAxis()));
		//</editor-fold>

		// Toggle intake position
		getOperatorRightBumper().whenPressed(new ToggleIntake(intakeFlipper));

		// Toggle shooter arm position
		getOperatorLeftBumper().whenPressed(new ToggleShooterArmPosition(shooterArm));

		// Set shooter rpm
		getOperatorXButton().whenPressed(new SetShooterRPM(shooter, NetworkTables.getShooterTargetRPM()));

		// Limelight align to target and shoot
		getDriverRightBumper().whileHeld(new LimelightShoot(drivetrain, limelight, shooter, storage, 2055));

		getOperatorLeftBumper().whenPressed(new InstantCommand());

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

		shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);
	}

	@Override
	public void autonomousInit() {
		scheduler.scheduleDefaultCommand(new PoseEstimation(drivetrain), TIME_OFFSET, 10);
		drivetrain.setBrake(); // Test, maybe bad idea
		shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);

		switch ((int) NetworkTables.getAutoChooser()) {
			case 0:
				autonomousCommand = new TaxiAuto(auto, drivetrain);
				break;

			case 1:
				autonomousCommand = new TwoCargoAutoDrive(auto, drivetrain, limelight);
				break;

			case 2:
				autonomousCommand = new TwoCargoAutoNoSA(auto, drivetrain, intake, storage, shooter, intakeFlipper, limelight, 2050);
				break;

			case 3:
				autonomousCommand = new TwoCargoAuto(auto, drivetrain, intake, storage, shooterArm, shooter, intakeFlipper, limelight, 2050);
				break;

			default:
				break;
		}

		if (autonomousCommand != null) {
			this.autoTask = scheduler.scheduleDefaultCommand(autonomousCommand, TIME_OFFSET, 10);
		}
	}

	@Override
	public void disabledInit(){}

	@Override
	public void teleopInit() {
		autonomousCommand.cancel();
		drivetrain.setBrake();
	}

	@Override
	public void testInit() {
		drivetrain.setCoast();
		autonomousCommand.cancel();
	}
}
