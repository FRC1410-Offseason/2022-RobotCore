package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.looped.*;
import frc.robot.commands.actions.*;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.subsystems.*;

import static frc.robotmap.IDs.PRESSURE_SENSOR;

public class Robot extends ScheduledRobot {

	private final String[] autoList = {"Taxi", "2Cargo", "3CargoTerminal", "3CargoUpRight", "4Cargo", "5Cargo"};
	private final AnalogInput pressure = new AnalogInput(PRESSURE_SENSOR);

	private Robot() {
		super(1000);
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

    private final TestSubsystem testSubsystem = new TestSubsystem();

	@Override
	public void registerControls() {
        
        //TODO: Uncomment Post-Testing
        //Drivetrain Default Command
		scheduler.scheduleDefaultCommand(new TankDrive(drivetrain, getDriverLeftYAxis(), getDriverRightYAxis()));
		// //Elevator Default Command
		// scheduler.scheduleDefaultCommand(new RunElevator(elevator, getOperatorLeftYAxis()));
		// //Winch Default Command
		// scheduler.scheduleDefaultCommand(new RunWinch(winch, getOperatorRightYAxis()));
		// //Intake Default Command
		// scheduler.scheduleDefaultCommand(new RunIntake(intake, getOperatorRightTrigger()));
		// //Outtake
		// scheduler.scheduleDefaultCommand(new ReverseIntake(intake, getOperatorLeftTrigger()).alongWith(new ReverseStorage(storage)));

		// //scheduler.scheduleCommand(); //TODO: Add shooter arm incrementing

		// getDriverRightBumper(); //TODO: Auto align and shoot

		// getOperatorLeftBumper().whileHeld(new ToggleShooterArm(shooterArm)); //TODO: Make this when pressed
		// getOperatorRightBumper().whileHeld(new ToggleIntake(intake)); //TODO: Make this toggle when pressed
		// getOperatorXButton(); //TODO: Make this toggle when pressed & add adaptive shooter RPM
		// getOperatorYButton().whileHeld(new RunStorage(storage));

        getDriverAButton().whenPressed(new TestActionCommand(testSubsystem));
        getDriverBButton().toggleWhenPressed(new TestLoopedCommand(testSubsystem));

        scheduler.scheduleDefaultCommand(new TestLoopedCommand(testSubsystem));
        
        scheduler.enableDebugTelemetry();        
	}

	@Override
	public void robotInit() {
		NetworkTables.setAutoList(autoList);
		NetworkTables.setCorrectColor(DriverStation.getAlliance().toString());
		NetworkTables.setPressure(pressure);
	}
}
