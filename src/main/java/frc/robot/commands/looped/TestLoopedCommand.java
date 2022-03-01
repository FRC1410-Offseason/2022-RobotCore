package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSubsystem;

public class TestLoopedCommand extends CommandBase {
    
    private final TestSubsystem subsystem;

    public TestLoopedCommand(TestSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("INITIALIZING LOOPED COMMAND");
    }

    @Override
    public void execute() {
        System.out.println("EXECUTING LOOPED COMMAND");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDING LOOPED COMMAND, Interrupted: " + interrupted);
    }
}
