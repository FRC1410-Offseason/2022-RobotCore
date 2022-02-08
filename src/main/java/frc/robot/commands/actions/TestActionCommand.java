package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestActionCommand extends CommandBase {
    @Override
	public void initialize() {
        System.out.println("Initializing Action Command");
	}

    @Override
	public void execute() {
        System.out.println("Executing Action Command");
	}

	@Override
	public boolean isFinished() {
		return true;
	}

    @Override
	public void end(boolean interrupted) {
        System.out.println("Ending Action Command, Interrupted: " + interrupted);
	}
}
