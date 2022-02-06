package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.looped.*;
import frc.robot.commands.actions.*;
import frc.robot.commands.grouped.FiveCargoAuto;
import frc.robot.framework.control.ControlScheme;
import frc.robot.framework.scheduler.EnqueuedTask;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.framework.scheduler.TaskScheduler;
import frc.robot.subsystems.*;
import frc.robot.util.Trajectories;
import static frc.robotmap.Tuning.*;

import static frc.robotmap.IDs.PRESSURE_SENSOR;

public class Robot extends ScheduledRobot implements ControlScheme {

	private final String[] autoList = {"Taxi", "2Cargo", "3CargoTerminal", "3CargoUpRight", "4Cargo", "5Cargo"};
	private final AnalogInput pressure = new AnalogInput(PRESSURE_SENSOR);
	private EnqueuedTask autoTask = null;

	private Robot() {
		super(20);
	}

	public static void main(String[] args) {
		RobotBase.startRobot(Robot::new);
	}

	private final Drivetrain drivetrain = new Drivetrain();
	private final Elevator elevator = new Elevator();
	private final Intake intake = new Intake();
	private final Shooter shooter = new Shooter();
	private final ShooterArm shooterArm = new ShooterArm();
	private final Storage storage = new Storage(DriverStation.getAlliance());
	private final Winch winch = new Winch();
	private final Trajectories auto = new Trajectories(this.drivetrain);

	@Override
	public TaskScheduler getScheduler() {
		return scheduler;
	}

	@Override
	public void registerControls() {
		scheduler.scheduleCommand(new TankDrive(drivetrain, getDriverLeftYAxis(), getDriverRightYAxis())); //Elevator Default Command
		scheduler.scheduleCommand(new RunElevator(elevator, getOperatorLeftYAxis())); //Elevator Default Command
		scheduler.scheduleCommand(new RunWinch(winch, getOperatorRightYAxis())); //Winch Default Command
		scheduler.scheduleCommand(new RunIntake(intake, getOperatorRightTrigger())); //Intake Default Command
		scheduler.scheduleCommand(new ReverseIntake(intake, getOperatorLeftTrigger()).alongWith(new ReverseStorage(storage))); //Outtake

		//scheduler.scheduleCommand(); //To Do: Add shooter arm incrementing

		getDriverRightBumper(); //To Do: Auto align and shoot
		getOperatorLeftBumper(); //To Do: Toggle shooter arm position
		getOperatorRightBumper().whileHeld(new ToggleIntake(intake)); //To Do: Make this toggle when pressed
		getOperatorXButton(); //To Do: Make this toggle when pressed & add adaptive shooter RPM
		getOperatorYButton().whileHeld(new RunStorage(storage));
	}

	@Override
	public void robotInit() {
		NetworkTables.setAutoList(autoList);
		NetworkTables.setCorrectColor(DriverStation.getAlliance().toString());
		NetworkTables.setPressure(pressure);
		if (RobotBase.isReal()) scheduler.scheduleCommand(new PoseEstimation(drivetrain), TIME_OFFSET, DT);
		if (RobotBase.isSimulation()) scheduler.scheduleCommand(new DrivetrainSimulation(drivetrain), TIME_OFFSET, DT);
	}

	@Override
	public void autonomousInit() {
		drivetrain.setBrake(); // Test, maybe bad idea
		Command autonomousCommand = null;
        if (NetworkTables.getAutoChooser() == 6) autonomousCommand = new FiveCargoAuto(auto, intake, shooter, shooterArm, storage);
        if (autonomousCommand != null) this.autoTask = scheduler.scheduleCommand(autonomousCommand, TIME_OFFSET, DT);

		// if (NetworkTables.getAutoChooser() == 1) autonomousCommand = auto.Taxi();
        // else if (NetworkTables.getAutoChooser() == 2) autonomousCommand = auto.UpperCargoShoot2();
        // else if (NetworkTables.getAutoChooser() == 3) autonomousCommand = auto.Terminal3Cargo();
        // else if (NetworkTables.getAutoChooser() == 4) autonomousCommand = auto.UpRight3Cargo();
        // else if (NetworkTables.getAutoChooser() == 5) autonomousCommand = auto.UpRightTerminal4Cargo();
		// else if (NetworkTables.getAutoChooser() == 6) autonomousCommand = auto.FiveCargo();
	}

	@Override
	public void disabledInit(){}
}
