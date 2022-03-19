package frc.robot.commands.grouped;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robotmap.Constants.*;
import frc.robot.commands.actions.*;
import frc.robot.subsystems.*;

import java.util.ArrayList;

public class LimelightShoot extends SequentialCommandGroup {

    public LimelightShoot(Drivetrain drivetrain, IntakeFlipper intakeFlipper, Limelight limelight, Shooter shooter, ShooterArm shooterArm, Storage storage, NetworkTableEntry RPM) {
		// TODO: Add calculation for desired exit velocity of cargo

		ArrayList<Command> toRun = new ArrayList<>();

		toRun.add(new ParallelCommandGroup(
			new RetractIntake(intakeFlipper),
			new ShooterSpinup(shooter, RPM.getDouble(0)),
			new LimelightAnglePID(limelight, drivetrain),
			new RunStorageForTime(storage, 0.3, -1)));
		toRun.add(new Shoot(shooter, storage));

		addCommands(toRun.toArray(Command[]::new));
    }
}
