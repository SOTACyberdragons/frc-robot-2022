// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

// TODO Need to cleanup scoping issues and tune properly.
public class TurnWithGyro extends CommandBase {
  public static double rotationAmount;
  public static double startHeading;
  public static double targetHeading;

  // PID Constansts
  private static final double kP = .03; // Power
  private static final double kI = .0075; // Ease in sensitivity
  private static final double kD = 0; // Smoothing
  private static final double kF = 0;

  public static PIDController m_pidController = new PIDController(kP, kI, kD);

  /** Creates a new TurnWithGyro. */
  public TurnWithGyro(double angleInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    rotationAmount = angleInput;
    addRequirements(Robot.m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startHeading = Robot.m_robotDrive.getAngle();
    targetHeading = startHeading + rotationAmount;
    m_pidController.setSetpoint(targetHeading);
    m_pidController.setTolerance(1, .5);
    SmartDashboard.putNumber("Target heading: ", targetHeading);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidOutput = MathUtil.clamp((m_pidController.calculate(Robot.m_robotDrive.getAngle()) + kF), -0.4, 0.4);
    Robot.m_robotDrive.m_drive(0, -pidOutput);
    SmartDashboard.putNumber("PID Output: ", m_pidController.calculate(Robot.m_robotDrive.getAngle()));    
    SmartDashboard.putNumber("PID Clamped: ", pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint(); 
  }
}