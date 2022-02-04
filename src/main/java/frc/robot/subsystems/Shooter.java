// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robotmap.IDs.*;
import static frc.robotmap.Tuning.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import frc.robot.framework.subsystem.SubsystemBase;

public class Shooter extends SubsystemBase {
    //Declare Motors
    private final CANSparkMax leftMotor = new CANSparkMax(SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);

    //Grab Encoders From Motors
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    //Grab PID Controllers
    private final SparkMaxPIDController leftController = leftMotor.getPIDController();
    private final SparkMaxPIDController rightController = rightMotor.getPIDController();

    private double target = 0;

    /** Creates a new Shooter. */
    public Shooter() {
        //Configure Motors
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightMotor.setInverted(true);

        //Configure PID controller outputs
        leftController.setOutputRange(-1, 1);
        rightController.setOutputRange(-1, 1);

        //Set PID loops to default values from the tuning file
        setLeftPID(
                SHOOTER_LEFT_kP,
                SHOOTER_LEFT_kI,
                SHOOTER_LEFT_kD,
                SHOOTER_LEFT_kFF
        );
        setRightPID(
                SHOOTER_RIGHT_kP,
                SHOOTER_RIGHT_kI,
                SHOOTER_RIGHT_kD,
                SHOOTER_RIGHT_kFF
        );
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //TODO: Possibly write code here to send velocity values to Network Tables
    }

    public void setLeftPID(double P, double I, double D) {
        this.setLeftPID(P, I, D, 0);
    }

    /**
     * Set the PID constants for the left controller
     * @param P proportional gain
     * @param I integral gain
     * @param D derivative gain
     * @param FF feed-forward gain
     */
    public void setLeftPID(double P, double I, double D, double FF) {
        leftController.setP(P);
        leftController.setI(I);
        leftController.setD(D);
        leftController.setFF(FF);
    }

    public void setRightPID(double P, double I, double D) {
        this.setRightPID(P, I, D, 0);
    }

    /**
     * Set PID Constants for the right controller
     * @param P proportional gain
     * @param I integral gain
     * @param D derivative gain
     * @param FF feed-forward gain
     */
    public void setRightPID(double P, double I, double D, double FF) {
        rightController.setP(P);
        rightController.setI(I);
        rightController.setD(D);
        rightController.setFF(FF);
    }


    /**
     * Sets the target velocities for the two NEOs in revolutions per minute
     * @param RPM speed of motors in revolutions per minute
     */
    public void setSpeeds(double RPM) {
        this.target = RPM;
        leftController.setReference(this.target, CANSparkMax.ControlType.kVelocity);
        rightController.setReference(this.target, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Returns the current reference speed for the shooter mechanism
     * @return The target speed (RPM)
     */
    public double getSpeed() {
        return this.target;
    }

    /**
     * Returns the velocity of the left motor specifically
     * @return The velocity of the left motor (RPM)
     */
    public double getLeftVel() {
        return leftEncoder.getVelocity();
    }

    /**
     * Returns the velocity of the right motor specifically
     * @return The velocity of the right motor (RPM)
     */
    public double getRightVel() {
        return rightEncoder.getVelocity();
    }

}