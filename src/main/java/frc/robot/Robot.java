package frc.robot;

import frc.robot.commands.actions.SetShooterArmAngle;
import frc.robot.commands.looped.DrivetrainSimulation;
import frc.robot.commands.looped.PoseEstimation;
import frc.robot.framework.control.ControlScheme;
import frc.robot.framework.scheduler.EnqueuedTask;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.framework.scheduler.TaskScheduler;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.*;
import frc.robot.util.Trajectories;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.*;

public class Robot extends ScheduledRobot implements ControlScheme {
	public static void main(String[] args) {
		RobotBase.startRobot(Robot::new);
	}

	private final ShooterArm shooterArm = new ShooterArm();
	private final Drivetrain drivetrain = new Drivetrain();
	private final String[] autoList = {"Taxi","2Cargo","3CargoTerminal","3CargoUpRight","4Cargo","5Cargo"};
	private final AnalogInput pressure = new AnalogInput(PRESSURE_SENSOR);
	private final Trajectories auto = new Trajectories(drivetrain);
	private EnqueuedTask autoTask = null;

	@Override
	public TaskScheduler getScheduler() {
		return scheduler;
	}

	@Override
    public void registerControls() {
		/*
		For Testing
		 */
		getDriverAButton().whileHeld(new SetShooterArmAngle(shooterArm, 50));
		getDriverBButton().whileHeld(new SetShooterArmAngle(shooterArm, 20));
		getDriverXButton().whileHeld(new SetShooterArmAngle(shooterArm, 0));
    }

	private Robot() {
		super(20);
	}

	@Override
	public void robotInit() {
		NetworkTables.networkTables();
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
        if (NetworkTables.getAutoChooser() == 1) autonomousCommand = auto.Taxi();
        else if (NetworkTables.getAutoChooser() == 2) autonomousCommand = auto.UpperCargoShoot2();
        else if (NetworkTables.getAutoChooser() == 3) autonomousCommand = auto.Terminal3Cargo();
        else if (NetworkTables.getAutoChooser() == 4) autonomousCommand = auto.UpRight3Cargo();
        else if (NetworkTables.getAutoChooser() == 5) autonomousCommand = auto.UpRightTerminal4Cargo();
        else if (NetworkTables.getAutoChooser() == 6) autonomousCommand = auto.FiveCargo();
		if (autonomousCommand != null) this.autoTask = scheduler.scheduleCommand(autonomousCommand, TIME_OFFSET, DT);
	}
}
