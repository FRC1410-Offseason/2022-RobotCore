package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;


public class TestCommand extends CommandBase {
    private final ExampleSubsystem exampleSubsystem;

    public TestCommand(ExampleSubsystem exampleSubsystem) {
        this.exampleSubsystem = exampleSubsystem;
        System.out.println("Scheduled");
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(exampleSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Init");
    }

    @Override
    public void execute() {
        System.out.println("Exe");
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("End");
    }
}
