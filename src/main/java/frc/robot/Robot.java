package frc.robot;

import edu.wpi.first.wpilibj.*;
import frc.robot.commands.actions.ReverseIntake;
import frc.robot.commands.actions.RunIntake;
import frc.robot.commands.actions.ToggleIntake;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.subsystems.Intake;

public class Robot extends ScheduledRobot {
	private final Intake intake = new Intake();

	public static void main(String[] args) {
		RobotBase.startRobot(Robot::new);
	}

    @Override
    public void registerControls() {
        OperatorAButton.whenPressed(new ToggleIntake(intake));
		OperatorXButton.whileHeld(new RunIntake(intake));
		OperatorYButton.whileHeld(new ReverseIntake(intake));
    }

	private Robot() {
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
	}
}
