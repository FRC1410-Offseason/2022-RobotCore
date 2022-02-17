package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.looped.*;
import frc.robot.commands.actions.*;
import frc.robot.commands.grouped.*;
import frc.robot.framework.scheduler.EnqueuedTask;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.framework.scheduler.TaskScheduler;
import frc.robot.subsystems.*;
import frc.robot.util.Trajectories;
import static frc.robotmap.Tuning.*;

import static frc.robotmap.IDs.PRESSURE_SENSOR;

public class Robot extends ScheduledRobot {

	private final String[] autoList = {"Taxi", "2Cargo", "3CargoTerminal", "3CargoUpRight", "4Cargo", "5Cargo"};
	private final AnalogInput pressure = new AnalogInput(PRESSURE_SENSOR);
	private EnqueuedTask autoTask = null;

	public static void main(String[] args) {RobotBase.startRobot(Robot::new);}
	private Robot() {super(20);}

	private final Drivetrain drivetrain = new Drivetrain();
	private final Elevator elevator = new Elevator();
	private final Intake intake = new Intake();
	private final Shooter shooter = new Shooter();
	private final ShooterArm shooterArm = new ShooterArm();
	private final Storage storage = new Storage(DriverStation.getAlliance());
	private final Winch winch = new Winch();
	private final Limelight limelight = new Limelight();
	private final Trajectories auto = new Trajectories(drivetrain);

	@Override
	public TaskScheduler getScheduler() {return scheduler;}

	@Override
	public void registerControls() {
		scheduler.scheduleCommand(new TankDrive(drivetrain, getDriverLeftYAxis(), getDriverRightYAxis())); // Elevator Default Command
		scheduler.scheduleCommand(new RunElevator(elevator, getOperatorLeftYAxis())); // Elevator Default Command
		scheduler.scheduleCommand(new RunWinch(winch, getOperatorRightYAxis())); // Winch Default Command
		scheduler.scheduleCommand(new RunIntake(intake, getOperatorRightTrigger())); // Intake Default Command

		getDriverRightBumper().whileHeld(new LimelightShoot(drivetrain, limelight, shooter, storage, shooterArm));
		getOperatorRightBumper().whileHeld(new ToggleIntake(intake)); // To Do: Make this toggle when pressed

		getOperatorDPadUp().whenPressed(new IncrementShooterArmAngle(shooterArm));
		getOperatorDPadDown().whenPressed(new DecrementShooterArmAngle(shooterArm));
	}

	@Override
	public void robotInit() {
		NetworkTables.setAutoList(autoList);
		NetworkTables.setCorrectColor(DriverStation.getAlliance().toString());
		NetworkTables.setPressure(pressure);
		if (RobotBase.isReal()) scheduler.scheduleCommand(new PoseEstimation(drivetrain, limelight, shooterArm), TIME_OFFSET, DT);
		if (RobotBase.isSimulation()) scheduler.scheduleCommand(new DrivetrainSimulation(drivetrain), TIME_OFFSET, DT);
	}

	@Override
	public void autonomousInit() {
		drivetrain.setBrake(); // Test, maybe bad idea
		Command autonomousCommand = null;

		switch ((int)NetworkTables.getAutoChooser()) {
			case 3:
				autonomousCommand = new ThreeCargoAutoClose(auto, intake, shooter, shooterArm, storage);
				break;

			case 4:
				autonomousCommand = new ThreeCargoTerminalAuto(auto, intake, shooter, shooterArm, storage);
				break;

			case 5:
				autonomousCommand = new FourCargoAuto(auto, intake, shooter, shooterArm, storage);
				break;

			case 6:
				autonomousCommand = new FiveCargoAuto(auto, intake, shooter, shooterArm, storage);
				break;

			default:
				break;
		}

		if (autonomousCommand != null) this.autoTask = scheduler.scheduleCommand(autonomousCommand, TIME_OFFSET, DT);
	}

	@Override
	public void disabledInit(){}
}
