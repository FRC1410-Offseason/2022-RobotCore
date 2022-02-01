package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.framework.scheduler.ScheduledRobot;
import static frc.robotmap.IDs.*;

public class TestRobot extends ScheduledRobot {
	private final String[] autoList = {"Taxi","2Cargo","3CargoTerminal","3CargoUpRight","4Cargo","5Cargo"};
	private final AnalogInput pressure = new AnalogInput(PRESSURE_SENSOR);
	public static void main(String[] args) {
		RobotBase.startRobot(TestRobot::new);
	}

    @Override
    public void registerControls() {
        
    }

	private TestRobot() {
		super(20);
	}

	private final Timer timer = new Timer();
	private int ticks = 0;

	@Override
	public void robotInit() {
		timer.start();
		scheduler.queuePeriodic(() -> {
			ticks++;

			if (timer.get() >= 5) {
				timer.reset();
				System.out.println(ticks);
				ticks = 0;
			}
		}, 2);
		NetworkTables.networkTables();
        NetworkTables.setAutoList(autoList);
		NetworkTables.setCorrectColor(DriverStation.getAlliance().toString());
		NetworkTables.setPressure(pressure);
	}
}
