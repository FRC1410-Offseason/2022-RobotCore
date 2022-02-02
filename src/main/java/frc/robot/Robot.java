package frc.robot;

import edu.wpi.first.wpilibj.*;

import frc.robot.commands.actions.SetShooterArmAngle;
import frc.robot.framework.control.ControlScheme;
import frc.robot.framework.scheduler.ScheduledRobot;
import frc.robot.framework.scheduler.TaskScheduler;
import frc.robot.subsystems.ShooterArm;

public class Robot extends ScheduledRobot implements ControlScheme {
	public static void main(String[] args) {
		RobotBase.startRobot(Robot::new);
	}

	private final ShooterArm shooterArm = new ShooterArm();

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
}
