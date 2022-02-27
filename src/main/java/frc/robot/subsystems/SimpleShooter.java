// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.utils.TalonFXConfig;

public class SimpleShooter extends SubsystemBase {

    // Create Falcon 500 motor objects
    public WPI_TalonFX leftMotor, rightMotor;

    // Falcon 500 faults objects
    private Faults faults = new Faults();

    // Moto speed containers
    public static double m_leftMotorSpeed;
    public static double m_rightMotorSpeed;

    // Some Talon specific variables
    public final static int kTimeoutMs = 30;
    public static final double kSensorUnitsPerRotation = 2048;
    public static final double gearRatio = 1;

    /** Creates a new SimpleShooter. */
    public SimpleShooter() {

        // Assign the motor objects
        leftMotor = TalonFXConfig.generateDefaultTalon(RobotMap.LEFT_SHOOTER_MOTOR);
        rightMotor = TalonFXConfig.generateDefaultTalon(RobotMap.RIGHT_SHOOTER_MOTOR);

        // Reset any embedded the motor configurations
        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();

        // Set the motor encoder's sensor phase (Don't change)
        leftMotor.setSensorPhase(false);
        rightMotor.setSensorPhase(false);

        // Make sure the motors are turning the right direction
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        // Behavior of the motors under 0 power. Brake or coast
        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);

        // We make the left a follower, so all commands need to be sent to the right
        // motor
        leftMotor.follow(rightMotor);

        // Final clean up on Talon data
        zeroSensors();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Shooter Velocity: ", getVelocity());
    }

    /* Zero all sensors on Talons */
    public void zeroSensors() {
        leftMotor.getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);
        rightMotor.getSensorCollection().setIntegratedSensorPosition(0, kTimeoutMs);
        System.out.println("[Integrated Sensors] All sensors are zeroed.\n");
    }

    public double getVelocity() {
        double ticksPerSample = leftMotor.getSelectedSensorVelocity();
        double ticksPerSecond = ticksPerSample * 10;
        double ticksPerMinute = ticksPerSecond * 60;
        double rpm = ticksPerMinute / kSensorUnitsPerRotation;
        return rpm;
    }
}