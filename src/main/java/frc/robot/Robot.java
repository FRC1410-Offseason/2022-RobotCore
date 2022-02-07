package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.framework.control.ControlScheme;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.framework.scheduler.TaskScheduler;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.looped.RunElevator;

import static frc.robotmap.IDs.PRESSURE_SENSOR;

public class Robot extends ScheduledRobot implements ControlScheme {

	private final Elevator elevator = new Elevator();

	private final String[] autoList = {"Taxi", "2Cargo", "3CargoTerminal", "3CargoUpRight", "4Cargo", "5Cargo"};
	private final AnalogInput pressure = new AnalogInput(PRESSURE_SENSOR);

	private Robot() {
		super(20);
	}

	public static void main(String[] args) {
		RobotBase.startRobot(Robot::new);
	}

	@Override
	public TaskScheduler getScheduler() {
		return scheduler;
	}

	@Override
	public void registerControls() {
		getScheduler().scheduleCommand(new RunElevator(elevator, getOperatorLeftYAxis()));
	}

	@Override
	public void robotInit() {
		NetworkTables.networkTables();
		NetworkTables.setAutoList(autoList);
		NetworkTables.setCorrectColor(DriverStation.getAlliance().toString());
		NetworkTables.setPressure(pressure);
	}
}
