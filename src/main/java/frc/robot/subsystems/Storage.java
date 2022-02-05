package frc.robot.subsystems;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    private final Alliance currentAlliance;
//    private boolean lineBroken = false;
//    private boolean ballDetected = false;
    private ColorSensorStatus currentColor = null;
    private BallStatus ballStatus = null;

    public enum ColorSensorStatus {
        ALLIANCE,
        NOT_ALLIANCE
    }

    public enum BallStatus {
        INDEXING,
        INDEXED
    }

    public Storage(Alliance currentAlliance) {
        colorMatcher.addColorMatch(BLUE_TARGET);
        colorMatcher.addColorMatch(RED_TARGET);
        this.currentAlliance = currentAlliance;
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
    public ColorSensorStatus getCurrentColor() {
        return currentColor;
    }

    /**
     * Returns the current value that the line break is reporting
     * @return true if there is no cargo, false if there is
     */
    public boolean getLineBreakState() {
        return lineBreak.get();
    }
    
    /**
     * Runs every robot cycle
     * Line Broken --> Motor runs & ball in storage
     * Line !Broken --> Motor stops & if ball in storage --> Update color
     */
    @Override
    public void periodic() {
        ColorMatchResult colorMatchResult = colorMatcher.matchClosestColor(colorSensor.getColor());

        if (currentAlliance == Alliance.Blue) {
            if (colorMatchResult.color.equals(BLUE_TARGET)) {
                currentColor = ColorSensorStatus.ALLIANCE;
            } else {
                currentColor = ColorSensorStatus.NOT_ALLIANCE;
            }
        } else {
            if (colorMatchResult.color.equals(RED_TARGET)) {
                currentColor = ColorSensorStatus.ALLIANCE;
            } else {
                currentColor = ColorSensorStatus.NOT_ALLIANCE;
            }
        }

        if (lineBreak.get()) {
            if (ballStatus == BallStatus.INDEXING) {
                ballStatus = BallStatus.INDEXED;
            }
        } else {
            ballStatus = BallStatus.INDEXING;
        }

        if (!lineBreak.get()) {
            runStorage(STORAGE_INDEX_SPEED);
        } else {
            runStorage(0);
            if (ballStatus == BallStatus.INDEXED) {
                if (currentColor == ColorSensorStatus.NOT_ALLIANCE) {
                    //TODO: Do outtake here
                }
            }
        }
    }
}

