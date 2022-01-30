package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TriggerCommand;
import frc.robot.commands.actions.RunIntake;
import frc.robot.commands.actions.ToggleIntake;
import frc.robot.control.input.Axis;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.*;

public class OI {

    public XboxController DriverController = new XboxController(DRIVER_CONTROLLER_PORT);
    public XboxController OperatorController = new XboxController(OPERATOR_CONTROLLER_PORT);

    private final Axis DriverLeftXAxis = new Axis(DriverController, AXIS_ID.LEFT_X, DRIVER_DEADZONE_VALUE);
    private final Axis DriverRightXAxis = new Axis(DriverController, AXIS_ID.LEFT_Y, DRIVER_DEADZONE_VALUE);
    private final Axis DriverLeftYAxis = new Axis(DriverController, AXIS_ID.RIGHT_X, DRIVER_DEADZONE_VALUE);
    private final Axis DriverRightYAxis = new Axis(DriverController, AXIS_ID.RIGHT_Y, DRIVER_DEADZONE_VALUE);
    private final Axis DriverLeftTrigger = new Axis(DriverController, AXIS_ID.LEFT_TRIGGER, DRIVER_DEADZONE_VALUE);
    private final Axis DriverRightTrigger = new Axis(DriverController, AXIS_ID.RIGHT_TRIGGER, DRIVER_DEADZONE_VALUE);

    private final Axis OperatorLeftXAxis = new Axis(OperatorController, AXIS_ID.LEFT_X, OPERATOR_DEADZONE_VALUE);
    private final Axis OperatorRightXAxis = new Axis(OperatorController, AXIS_ID.RIGHT_X, OPERATOR_DEADZONE_VALUE);
    private final Axis OperatorLeftYAxis = new Axis(OperatorController, AXIS_ID.LEFT_Y, OPERATOR_DEADZONE_VALUE);
    private final Axis OperatorRightYAxis = new Axis(OperatorController, AXIS_ID.RIGHT_Y, OPERATOR_DEADZONE_VALUE);
    private final Axis OperatorLeftTrigger = new Axis(OperatorController, AXIS_ID.LEFT_TRIGGER, OPERATOR_DEADZONE_VALUE);
    private final Axis OperatorRightTrigger = new Axis(OperatorController, AXIS_ID.RIGHT_TRIGGER, OPERATOR_DEADZONE_VALUE);

    private final JoystickButton DriverAButton = new JoystickButton(DriverController, A_BUTTON_ID);
    private final JoystickButton DriverBButton = new JoystickButton(DriverController, B_BUTTON_ID);
    private final JoystickButton DriverXButton = new JoystickButton(DriverController, X_BUTTON_ID);
    private final JoystickButton DriverYButton = new JoystickButton(DriverController, Y_BUTTON_ID);
    private final JoystickButton DriverLeftBumper = new JoystickButton(DriverController, LEFT_BUMPER_ID);
    private final JoystickButton DriverRightBumper = new JoystickButton(DriverController, RIGHT_BUMPER_ID);
    private final JoystickButton DriverLeftStickButton = new JoystickButton(DriverController, LEFT_STICK_BUTTON_ID);
    private final JoystickButton DriverRightStickButton = new JoystickButton(DriverController, RIGHT_STICK_BUTTON_ID);

    private final JoystickButton OperatorAButton = new JoystickButton(OperatorController, A_BUTTON_ID);
    private final JoystickButton OperatorBButton = new JoystickButton(OperatorController, B_BUTTON_ID);
    private final JoystickButton OperatorXButton = new JoystickButton(OperatorController, X_BUTTON_ID);
    private final JoystickButton OperatorYButton = new JoystickButton(OperatorController, Y_BUTTON_ID);
    private final JoystickButton OperatorLeftBumper = new JoystickButton(OperatorController, LEFT_BUMPER_ID);
    private final JoystickButton OperatorRightBumper = new JoystickButton(OperatorController, RIGHT_BUMPER_ID);
    private final JoystickButton OperatorLeftStickButton = new JoystickButton(OperatorController, LEFT_STICK_BUTTON_ID);
    private final JoystickButton OperatorRightStickButton = new JoystickButton(OperatorController, RIGHT_STICK_BUTTON_ID);
    
    public OI() {}

    public void registerControls() {
        DriverXButton.whileHeld(new TriggerCommand(SubsystemEngine.getInstance().getSubsystem(ExampleSubsystem.class)));

        //INTAKE
        OperatorAButton.whenPressed(new ToggleIntake(SubsystemEngine.getInstance().getSubsystem(Intake.class)));
        OperatorXButton.whileHeld(new RunIntake(SubsystemEngine.getInstance().getSubsystem(Intake.class)));
    }
}
