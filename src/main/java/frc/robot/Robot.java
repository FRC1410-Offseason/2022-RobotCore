package frc.robot;

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
			default: throw new IllegalStateException("Unknown auto profile " + auto);
		}


		scheduler.scheduleDefaultCommand(autonomousCommand, TIME_OFFSET, 10, RobotMode.TELEOP, RobotMode.TEST);
		scheduler.debugDumpList();
	}

    @Override
	public void registerControls() {

		/**
		 * Intaking sequence:
		 * 	Run the intake at full speed
		 * 	Run the storage at full speed
		 * 	Run the shooter wheels backwards
		 */
		getOperatorYButton().whileHeld(new RunIntakeWithButton(intake, storage, shooter));
		getOperatorXButton().whileHeld(new RunIntakeWithButton(intake, storage, shooter));

		/**
		 * Sequences for toggling the robot position, the robot is either:
		 * 	In an intaking position, where the shooter arm is down and the intake is out
		 * 	In a shooting / scoring position, where the shooter arm is up and the intake is up
		 * 	In a climbing position, where the shooter arm is down but the intake is up (lowers the center of mass)
		 */
        getOperatorRightBumper().whenPressed(new RaiseShooterArmDelayed(shooterArm));
        getOperatorRightBumper().whenPressed(new RetractIntake(intakeFlipper));
		getOperatorLeftBumper().whenPressed(new LowerShooterArm(shooterArm));
        getOperatorLeftBumper().whenPressed(new ExtendIntakeDelayed(intakeFlipper));

		getOperatorDPadLeft().whenPressed(new LowerShooterArm(shooterArm));
		getOperatorDPadRight().whenPressed(new RaiseShooterArm(shooterArm));
        getOperatorDPadRight().whenPressed(new RetractIntake(intakeFlipper));

		/**
		 * Manually spin up the shooter wheels, in case the driver needs to take over
		 */
		getOperatorDPadUp().whenPressed(new SetShooterRPM(shooter, SHOOTER_LOW_HUB_RPM));
		getOperatorDPadDown().whenPressed(new SetShooterRPM(shooter, 0));

		// Sequence for scoring into the low hub
		getDriverAButton().whenPressed(new LowHubShoot(shooter, shooterArm, storage, SHOOTER_LOW_HUB_RPM));

		// Flip the direction that the drivetrain moves relative to the driver controller
        getDriverYButton().whenPressed(new FlipDrivetrain(drivetrain));

		// Telescoping arms are controlled independently on a button and a trigger for the driver
        getDriverLeftBumper().whileHeld(new RunLeftTAConstant(leftTA, TA_RAISE_SPEED));
        getDriverRightBumper().whileHeld(new RunRightTAConstant(rightTA, TA_RAISE_SPEED));
	}

	@Override
	public void teleopInit() {

		// Make sure the drivetrain is in brake mode
		drivetrain.setBrake();

		// The command that handles the intake flipper motors
		scheduler.scheduleDefaultCommand(new RunIntakeFlipper(intakeFlipper));

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
		scheduler.scheduleDefaultCommand(new RunIntakeFlipperWithAxis(intakeFlipper, getOperatorRightYAxis()));

		getOperatorXButton().whenPressed(new InstantCommand(() -> intakeFlipper.resetEncoders(INTAKE_UP_POSITION)));
		getOperatorYButton().whenPressed(new InstantCommand(() -> intakeFlipper.resetEncoders(INTAKE_DOWN_POSITION)));

		getOperatorDPadUp().whenPressed(new SetShooterRPM(shooter, SHOOTER_LOW_HUB_RPM));
		getOperatorDPadDown().whenPressed(new SetShooterRPM(shooter, 0));

		getOperatorAButton().whileHeld(new RunStorageConstant(storage, STORAGE_RUN_SPEED));

	}

	@Override
	public void testPeriodic() {
		System.out.println(intakeFlipper.getEncoderPosition());
	}
}
