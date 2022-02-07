package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestInstantCommand extends CommandBase {
    @Override
	public void initialize() {
        System.out.println("Initializing");
	}

    @Override
	public void execute() {
        System.out.println("Executing");
	}

	@Override
	public boolean isFinished() {
		return true;
	}

    @Override
	public void end(boolean interrupted) {
        System.out.println("Ending, Interrupted: " + interrupted);
	}
}
