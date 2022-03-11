package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.controllers.Axis;
import frc.robot.subsystems.RightTelescopingArm;

public class RunRightTA extends CommandBase {
    private final RightTelescopingArm arm;
    private final Axis axis;

    public RunRightTA(RightTelescopingArm arm, Axis axis) {
        this.arm = arm;
        this.axis = axis;

        addRequirements(arm);
    }

    @Override
	public void execute() {
		arm.set(-axis.getRaw());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		arm.set(0);
	}
    
}
