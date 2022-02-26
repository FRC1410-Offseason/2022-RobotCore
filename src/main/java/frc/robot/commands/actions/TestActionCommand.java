package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSubsystem;

public class TestActionCommand extends CommandBase {
    
    private final TestSubsystem subsystem;

    public TestActionCommand(TestSubsystem subsystem) {
        this.subsystem = subsystem;
    }
    
    @Override
    public void initialize() {
        System.out.println("INITIALIZING ACTION COMMAND");
    }

    @Override
    public void execute() {
        System.out.println("EXECUTING ACTION COMMAND");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDING ACTION COMMAND, Interrupted: " + interrupted);
    }
}
