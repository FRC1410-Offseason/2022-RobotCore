package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSubsystem;

//TODO: Delete this file
public class TestActionCommand extends CommandBase {
    
    private final TestSubsystem subsystem;

    public TestActionCommand(TestSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    
    @Override
	public void initialize() {
        System.out.println("Initializing Action Command");
	}

    @Override
	public void execute() {
        System.out.println("Executing Action Command, currently binding " + subsystem);
	}

	@Override
	public boolean isFinished() {
		return true;
	}

    @Override
	public void end(boolean interrupted) {
        System.out.println("Ending Action Command " + this + ", Interrupted: " + interrupted);
	}
}
