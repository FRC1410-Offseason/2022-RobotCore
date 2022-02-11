package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.looped.*;
import frc.robot.commands.actions.*;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.subsystems.*;

import static frc.robotmap.IDs.PRESSURE_SENSOR;
import static frc.robotmap.Constants.*;

public class Robot extends ScheduledRobot {

	private final String[] autoList = {"Taxi", "2Cargo", "3CargoTerminal", "3CargoUpRight", "4Cargo", "5Cargo"};
	private final AnalogInput pressure = new AnalogInput(PRESSURE_SENSOR);

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

	@Override
	public void registerControls() {
        
        //Drivetrain Default Command
		scheduler.scheduleCommand(new TankDrive(drivetrain, getDriverLeftYAxis(), getDriverRightYAxis()));
		//Elevator Default Command
		scheduler.scheduleCommand(new RunElevator(elevator, getOperatorLeftYAxis()));
		//Winch Default Command
		scheduler.scheduleCommand(new RunWinch(winch, getOperatorRightYAxis()));
		//Intake Default Command
		scheduler.scheduleCommand(new RunIntake(intake, getOperatorRightTrigger()));
		//Storage Default Command
		scheduler.scheduleCommand(new OuttakeHandler(shooter, shooterArm, storage));

		getOperatorDPadUp().whenPressed(new IncrementShooterArmAngle(shooterArm));
		getOperatorDPadDown().whenPressed(new DecrementShooterArmAngle(shooterArm));

		getDriverRightBumper(); //TODO: Auto align and shoot
		
		getOperatorRightBumper().whileHeld(new ToggleIntake(intake)); //TODO: Make this toggle when pressed
		getOperatorXButton(); //TODO: Make this toggle when pressed & add adaptive shooter RPM

		getDriverAButton().whenPressed(new SetShooterArmAngle(shooterArm, SHOOTER_ARM_MAX_ANGLE));
		getDriverBButton().whenPressed(new SetShooterArmAngle(shooterArm, SHOOTER_ARM_RESTING_ANGLE));
	}

	@Override
	public void robotInit() {
		NetworkTables.setAutoList(autoList);
		NetworkTables.setCorrectColor(DriverStation.getAlliance().toString());
		NetworkTables.setPressure(pressure);
	}
}
