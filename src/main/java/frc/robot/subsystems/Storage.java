package frc.robot.subsystems;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class Storage extends SubsystemBase {
    //Motor that runs the storage
    private final CANSparkMax motor = new CANSparkMax(STORAGE_MOTOR_ID, MotorType.kBrushless);
    //Color sensor
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    private final ColorMatch colorMatcher = new ColorMatch();
    //Line break sensor
    private final DigitalInput lineBreak = new DigitalInput(STORAGE_LINE_BREAK_PORT);

    private boolean lineBroken = false;
    private boolean ballDetected = false;
    private ColorSensorState currentColor = null;

    public enum ColorSensorState {
        BLUE,
        RED
    }

    public void storage() {
        colorMatcher.addColorMatch(BLUE_TARGET);
        colorMatcher.addColorMatch(RED_TARGET);
        motor.restoreFactoryDefaults();
    }

    /**
     * Runs the storage at a given speed
     * @param power double from -1 to 1
     */
    public void runStorage(double power) {
        motor.set(power);
    }

    /**
     * Returns the current state of the color sensor
     * @return either RED or BLUE
     */
    public ColorSensorState getCurrent_color() {
        return currentColor;
    }

    /**
     * Returns the current value that the line break is reporting
     * @return true if there is no cargo, false if there is
     */
    public boolean getLineBreakState() {
        return lineBroken;
    }
    
    /**
     * Runs every robot cycle
     * Line Broken --> Motor runs & ball in storage
     * Line !Broken --> Motor stops & if ball in storage --> Update color
     */
    @Override
    public void periodic() {
        lineBroken = !lineBreak.get();
        ColorMatchResult currentColor = colorMatcher.matchClosestColor(colorSensor.getColor());
        
        if (lineBroken) {
            motor.set(STORAGE_INDEX_SPEED);
            ballDetected = true;
        } else {
            motor.set(0);

            if (ballDetected) {
                if (currentColor.color.equals(BLUE_TARGET)) {
                    currentColor = ColorSensorState.BLUE;
                } else {
                    current_color = ColorSensorState.RED;
                }
                ballDetected = false;
            }
        }
    }
}

