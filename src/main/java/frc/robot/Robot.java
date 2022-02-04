package frc.robot;

import edu.wpi.first.wpilibj.*;

import frc.robot.commands.actions.SetShooterArmAngle;
import frc.robot.commands.actions.TestCommand;
import frc.robot.framework.control.ControlScheme;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.framework.scheduler.TaskScheduler;
import frc.robot.subsystems.*;
import static frc.robotmap.IDs.*;

public class Robot extends ScheduledRobot implements ControlScheme {
	private final String[] autoList = {"Taxi","2Cargo","3CargoTerminal","3CargoUpRight","4Cargo","5Cargo"};
	private final AnalogInput pressure = new AnalogInput(PRESSURE_SENSOR);

	public static void main(String[] args) {
		RobotBase.startRobot(Robot::new);
	}

	// private final ShooterArm shooterArm = new ShooterArm();
    private final ExampleSubsystem fuckySystem = new ExampleSubsystem();

	@Override
	public TaskScheduler getScheduler() {
		return scheduler;
	}

	@Override
    public void registerControls() {
		/*
		For Testing
		 */
		// getDriverAButton().whileHeld(new SetShooterArmAngle(shooterArm, 50));
		// getDriverBButton().whileHeld(new SetShooterArmAngle(shooterArm, 20));
		// getDriverXButton().whileHeld(new SetShooterArmAngle(shooterArm, 0));

        scheduler.scheduleCommand(new TestCommand(fuckySystem));

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
	}
}
