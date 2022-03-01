package frc.robot.commands.looped;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSubsystem;

public class TestLoopedCommand extends CommandBase {
    
    private final TestSubsystem subsystem;
    private final String inputString;

    public TestLoopedCommand(TestSubsystem subsystem, String inputString) {
        this.subsystem = subsystem;
        this.inputString = inputString;

        addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("INITIALIZING " + inputString + " LOOPED COMMAND");
    }

    @Override
    public void execute() {
        System.out.println("EXECUTING " + inputString + " LOOPED COMMAND");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDING " + inputString + " LOOPED COMMAND, Interrupted: " + interrupted);
    }
}
