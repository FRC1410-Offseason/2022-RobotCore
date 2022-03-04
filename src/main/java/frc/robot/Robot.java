package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
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
	private EnqueuedTask autoTask = null;
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
	public void registerControls() {
		getDriverRightBumper().whenPressed(new LimelightShoot(drivetrain, limelight, shooter, storage, 2055));
		getDriverLeftBumper().whenPressed(new LimelightAnglePID(limelight, drivetrain));
		getOperatorRightBumper().whileHeld(new ToggleIntake(intakeFlipper));

//		getDriverAButton().whenPressed(new RunCommand(() -> winch.lock(), winch));
//		getDriverXButton().whenPressed(new RunCommand(() -> winch.unlock(), winch));
	}

	@Override
	public void robotInit() {
		NetworkTables.setAutoList(autoList);
		NetworkTables.setCorrectColor(DriverStation.getAlliance().toString());
		NetworkTables.setPressure(pressure);
		drivetrain.setCoast();
	}

	@Override
	public void autonomousInit() {
		shooterArm.resetEncoder(SHOOTER_ARM_MAX_ANGLE);
		scheduler.scheduleDefaultCommand(new RunShooterArm(shooterArm));
		scheduler.scheduleDefaultCommand(new PoseEstimation(drivetrain), TIME_OFFSET, (long) 10);
		drivetrain.setBrake();

		switch ((int)NetworkTables.getAutoChooser()) {

			case 0:
				// autonomousCommand = new TaxiAuto(auto, drivetrain);

			case 1:
				// autonomousCommand = new TwoCargoAutoDrive(auto, drivetrain, limelight);

			case 2:
				// autonomousCommand = new TwoCargoAutoNoSA(auto, drivetrain, intake, storage, shooter, intakeFlipper, limelight, 2050);

			case 3:
				autonomousCommand = new TwoCargoAuto(auto, drivetrain, intake, storage, shooterArm, shooter, intakeFlipper, limelight, NetworkTables.getAutoRPM());

			default:
				break;
		}

//		if (autonomousCommand != null) this.autoTask = scheduler.scheduleDefaultCommand(autonomousCommand, TIME_OFFSET, (long) 10);
	}

	@Override
	public void teleopInit() {
		drivetrain.setBrake();

		scheduler.scheduleDefaultCommand(new TankDrive(drivetrain, getDriverLeftYAxis(), getDriverRightYAxis()));
//		scheduler.scheduleDefaultCommand(new RunElevator(elevator, getOperatorLeftYAxis()));
//		scheduler.scheduleDefaultCommand(new RunWinch(winch, getOperatorRightYAxis()));

		scheduler.scheduleDefaultCommand(new RunIntake(intake, storage, getOperatorRightTrigger()));
//		scheduler.scheduleDefaultCommand(new RunIntakeFlipper(intakeFlipper));
		scheduler.scheduleDefaultCommand(new RunIntakeFlipperWithAxis(intakeFlipper, getOperatorLeftYAxis()));



	}

	@Override
	public void testInit() {
		drivetrain.setCoast();
		autonomousCommand.cancel();
	}
}
