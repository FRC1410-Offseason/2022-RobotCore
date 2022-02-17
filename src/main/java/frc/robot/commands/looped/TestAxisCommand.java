package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.framework.control.controllers.Axis;

//TODO: Delete this file
public class TestAxisCommand extends CommandBase {
    
    private final Axis internalAxis;
    public TestAxisCommand(Axis internalAxis) {
        this.internalAxis = internalAxis;
    }
    @Override
	public void initialize() {
        System.out.println("Initializing Axis Command");
	}

    @Override
	public void execute() {
        System.out.println("Executing Axis Command");
        System.out.println("Raw Axis Value: " + internalAxis.getRaw());
        System.out.println("Deadzoned Axis Value: " + internalAxis.getDeadzoned());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

    @Override
	public void end(boolean interrupted) {
        System.out.println("Ending Axis Command, Interrupted: " + interrupted);
	}
}
