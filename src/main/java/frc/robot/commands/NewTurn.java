// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class NewTurn extends CommandBase {

    private static double rotationAmount;
    public static double startingAngle;
    public static double targetAngle;
    public static double currentAngle;

    public static double turnDirection = 1;

    // Motor characterization
    private static double kS = 0.8975;
    private static double kV = 0.0026249;
    private static double kA = 8.3993E-05;

    // Motor PID values
    private static double kP = 0.1084;
    private static double kI = 0;
    private static double kD = 0.0039948;

    // Setpoint Tolerance
    private static double kTurnTolerance = 2;

    // private static PIDController m_pidController;
    // private static SimpleMotorFeedforward m_feedForward;

    private static PIDController m_pidController = new PIDController(kP, kI, kD);
    private static SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(kS, kV, kA);

    /** Creates a new NewTurn. */
    public NewTurn(double angleInput) {
        rotationAmount = angleInput;

        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(RobotContainer.m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Get the initial robot position
        startingAngle = RobotContainer.m_drive.getRotation();

        // Define the endpoints
        targetAngle = startingAngle + rotationAmount;

        // Create the PID objects
        m_pidController.reset();

        // Setup the PID controller
        m_pidController.setSetpoint(targetAngle);
        m_pidController.setTolerance(kTurnTolerance);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentAngle = RobotContainer.m_drive.getRotation();

        double pidOutput = m_pidController.calculate(currentAngle, targetAngle);
        double powerOutput = ((pidOutput + m_feedForward.calculate(targetAngle)) / 12) - 0.25;

        // RobotContainer.m_drive.arcadeDrive(0, pidOutput +
        // -m_feedForward.calculate(targetAngle));
        RobotContainer.m_drive.arcadeDrive(0, -powerOutput);

        SmartDashboard.putNumber("rotationAmount", rotationAmount);
        SmartDashboard.putNumber("targetAngle", targetAngle);
        SmartDashboard.putNumber("currentAngle", currentAngle);
        SmartDashboard.putNumber("feedForward", m_feedForward.calculate(targetAngle));
        SmartDashboard.putNumber("pidOutput", pidOutput);
        SmartDashboard.putNumber("powerOutput", powerOutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_drive.arcadeDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_pidController.atSetpoint()) {
            SmartDashboard.putBoolean("Turning done", true);
            return true;
        } else {
            SmartDashboard.putBoolean("Turning done", false);
            return false;
        }
    }
}
