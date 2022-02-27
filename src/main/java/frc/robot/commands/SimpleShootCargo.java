// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.fasterxml.jackson.databind.introspect.AccessorNamingStrategy.Provider;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.subsystems.SimpleShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleShootCargo extends PIDCommand {

    // Motor characterization
    private static double kS = 0;
    private static double kV = 0;
    private static double kA = 0;

    private double velocityTolerance = 100;
    private static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    /** Creates a new SimpleShootCargo. */
    public SimpleShootCargo(double targetRPM, SimpleShooter m_shooter) {
        super(
                // The controller that the command will use
                new PIDController(0, 0, 0),
                // Close loop on shooter velocity
                m_shooter::getVelocity,
                // This should return the setpoint
                targetRPM,
                // Pipe output to turn robot
                output -> m_shooter.rightMotor.set(ControlMode.PercentOutput, output + feedforward.calculate(targetRPM)),
                // Require the shooter
                m_shooter);

        // Set the controller tolerance
        getController().setTolerance(velocityTolerance);
    }

    @Override
    public void execute() {
        // Do stuff ...
        if (getController().atSetpoint()) {
            System.out.println("At setpoint!");
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.m_simpleShooter.rightMotor.set(ControlMode.PercentOutput, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
