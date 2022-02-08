package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestLoopedCommand extends CommandBase {
    @Override
	public void initialize() {
        System.out.println("Initializing Looped Command");
	}

    @Override
	public void execute() {
        System.out.println("Executing Looped Command");
	}

	@Override
	public boolean isFinished() {
		return false;
	}

    @Override
	public void end(boolean interrupted) {
        System.out.println("Ending Looped Command, Interrupted: " + interrupted);
	}
}
