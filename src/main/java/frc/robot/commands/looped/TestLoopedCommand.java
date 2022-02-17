package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSubsystem;

//TODO: Delete this file
public class TestLoopedCommand extends CommandBase {

    private final TestSubsystem subsystem;

    public TestLoopedCommand(TestSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
	public void initialize() {
        System.out.println("Initializing Looped Command");
	}

    @Override
	public void execute() {
        System.out.println("Executing Looped Command, currently binding " + subsystem);
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
