// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
    // Create Falcon 500 motor objects
    public WPI_TalonSRX primaryPivotMotor, secondaryPivotMotor, primaryArmMotor, secondaryArmMotor;

    /*
     * Local, standardized power setting for this subsystem.
     */

    private double kArmSpeed = .25;
    private double kPivotSpeed = .25;

    public enum Direction {
        UP, DOWN
    };

    /** Creates a new NewClimber. */
    public Climber() {

        /*
         * Assign the pivot motor objects
         */

        primaryPivotMotor = new WPI_TalonSRX(RobotMap.PRIMARY_PIVOT_MOTOR);
        secondaryPivotMotor = new WPI_TalonSRX(RobotMap.SECONDARY_PIVOT_MOTOR);

        // Reset any embedded pivot motor configurations
        primaryPivotMotor.configFactoryDefault();
        secondaryPivotMotor.configFactoryDefault();

        // Set the pivot motor encoder's sensor phase (Don't change)
        primaryPivotMotor.setSensorPhase(false);
        secondaryPivotMotor.setSensorPhase(false);

        // Make sure the pivot motors are turning the right direction
        primaryPivotMotor.setInverted(false);
        secondaryPivotMotor.setInverted(true);

        // Behavior of the pivot motors under 0 power. Brake or coast
        primaryPivotMotor.setNeutralMode(NeutralMode.Brake);
        secondaryPivotMotor.setNeutralMode(NeutralMode.Brake);

        // We make the left a follower, so all commands need to be sent to the right
        // pivot motor
        secondaryPivotMotor.follow(primaryPivotMotor);

        /*
         * Assign the arm motor objects
         */

        primaryArmMotor = new WPI_TalonSRX(RobotMap.PRIMARY_ARM_MOTOR);
        secondaryArmMotor = new WPI_TalonSRX(RobotMap.SECONDARY_ARM_MOTOR);

        // Reset any embedded arm motor configurations
        primaryArmMotor.configFactoryDefault();
        secondaryArmMotor.configFactoryDefault();

        // The arm motors DON'T need to be inverted
        primaryArmMotor.setInverted(false);
        secondaryArmMotor.setInverted(false);

        // Behavior of the arm motors under 0 power. Brake or coast
        primaryArmMotor.setNeutralMode(NeutralMode.Brake);
        secondaryArmMotor.setNeutralMode(NeutralMode.Brake);

        // We make the left a follower, so all commands need to be sent to the right
        // arm motor
        secondaryArmMotor.follow(primaryArmMotor);
    }

    public void moveArms(String direction) {
        if (direction.equals("UP")) {
            primaryArmMotor.set(ControlMode.PercentOutput, kArmSpeed);
        } else if (direction.equals("DOWN")) {
            primaryArmMotor.set(ControlMode.PercentOutput, -kArmSpeed);
        }
    }

    public void stopArms() {
        primaryArmMotor.set(ControlMode.PercentOutput, 0);
    }

    public void pivotForward() {
        primaryPivotMotor.set(ControlMode.PercentOutput, kPivotSpeed);
    }

    public void pivotBackward() {
        primaryPivotMotor.set(ControlMode.PercentOutput, -kPivotSpeed);
    }

    public void stopPivot() {
        primaryPivotMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
