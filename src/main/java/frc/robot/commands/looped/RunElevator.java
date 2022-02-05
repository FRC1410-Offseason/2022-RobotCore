package frc.robot.commands.looped;

import static frc.robotmap.Constants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.Axis;
import frc.robot.subsystems.Elevator;


public class RunElevator extends CommandBase {

	private final Elevator elevator;
	private final Axis axis;
	private final LinearSystemLoop<N2, N1, N1> loop;

	private boolean shouldRun = false;

	private TrapezoidProfile.State goal;
	private TrapezoidProfile.State lpr;

	public RunElevator(Elevator elevator, Axis axis) {
		this.elevator = elevator;
		this.axis = axis;
		this.loop = elevator.getLoop();
		addRequirements(elevator);
	}

	@Override
	public void initialize() {
		loop.reset(VecBuilder.fill(elevator.getEncoderPosition(), elevator.getEncoderVelocity()));
		lpr = new TrapezoidProfile.State(elevator.getEncoderPosition(), elevator.getEncoderVelocity());
	}

	@Override
	public void execute() {
		shouldRun = Math.abs(axis.getDeadzoned()) < 0.1;

		if (!shouldRun) {
			elevator.setLock(DoubleSolenoid.Value.kReverse);
			double desiredSpeed = -axis.getDeadzoned() * ELEVATOR_MAX_VEL;
			if (desiredSpeed < -0.01) {
				goal = new TrapezoidProfile.State(ELEVATOR_MIN_POS, desiredSpeed);
			} else if (desiredSpeed > 0.01) {
				goal = new TrapezoidProfile.State(ELEVATOR_MAX_POS, desiredSpeed);
			}

			lpr = (new TrapezoidProfile(elevator.getConstraints(), goal, lpr)).calculate(DT);
			loop.setNextR(lpr.position, lpr.velocity);
			loop.correct(VecBuilder.fill(elevator.getEncoderPosition()));
			loop.predict(DT);

			elevator.setVoltage(loop.getU(0));
		} else {
			elevator.setVoltage(0);
			elevator.setLock(DoubleSolenoid.Value.kForward);
		}
	}

	@Override
	public boolean isFinished() {
		// TODO: Make this return true when this Command no longer needs to run execute()
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		System.out.println("End");
	}
}
