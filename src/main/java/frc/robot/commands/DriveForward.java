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
public class DriveForward extends CommandBase {
  public static double driveDistance;
  public static double startingDistance;
  public static double targetDistance;

  // PID Constansts
  private static final double kP = .25; // Power
  private static final double kI = .0075; // Ease in sensitivity
  private static final double kD = .125; // Smoothing
  private static final double kF = .1;
  
  public static PIDController m_pidController = new PIDController(kP, kI, kD);

  /** Creates a new DriveForward. */
  public DriveForward(double distanceInMeters) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveDistance = distanceInMeters;
    addRequirements(Robot.m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingDistance = Robot.m_robotDrive.getAverageDistance();
    targetDistance = startingDistance + driveDistance;
    m_pidController.setSetpoint(targetDistance);
    m_pidController.setTolerance(.05, .05);
    SmartDashboard.putNumber("Target distance: ", targetDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidOutput = MathUtil.clamp((m_pidController.calculate(Robot.m_robotDrive.getAverageDistance()) + kF), -0.5, 0.5);
    Robot.m_robotDrive.m_drive(pidOutput, 0);
    SmartDashboard.putNumber("PID Output: ", m_pidController.calculate(Robot.m_robotDrive.getAverageDistance()));    
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
