package frc.robot.commands.actions;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterArm;

import static frc.robotmap.Constants.DT;
import static frc.robotmap.Constants.SHOOTER_ARM_IS_FINISHED_THRESHOLD;


public class SetShooterArmAngle extends CommandBase {

	private final ShooterArm shooterArm;
	private final LinearSystemLoop<N2, N1, N1> armLoop;

	private TrapezoidProfile.State lpr;
	private TrapezoidProfile.State goal;

	public SetShooterArmAngle(ShooterArm shooterArm, double angle) {
		this.shooterArm = shooterArm;
		this.armLoop = shooterArm.getLoop();
		goal = new TrapezoidProfile.State(Units.degreesToRadians(angle), 0);
		// addRequirements(shooterArm);
	}

	@Override
	public void initialize() {
		shooterArm.releaseBrake();
		armLoop.reset(VecBuilder.fill(shooterArm.getEncoderPosition(), shooterArm.getEncoderVelocity()));
		lpr = new TrapezoidProfile.State(shooterArm.getEncoderPosition(), shooterArm.getEncoderVelocity());
	}

	@Override
	public void execute() {
		lpr = (new TrapezoidProfile(shooterArm.getConstraints(), goal, lpr)).calculate(DT);
		armLoop.setNextR(lpr.position, lpr.velocity);
		armLoop.correct(VecBuilder.fill(shooterArm.getEncoderPosition()));
		armLoop.predict(DT);
		shooterArm.setVoltage(armLoop.getU(0));
	}

	@Override
	public boolean isFinished() {
		return armLoop.getError(0) < Units.degreesToRadians(SHOOTER_ARM_IS_FINISHED_THRESHOLD);
	}

	@Override
	public void end(boolean interrupted) {
		shooterArm.setBrake();
		shooterArm.setVoltage(0);
	}
}
