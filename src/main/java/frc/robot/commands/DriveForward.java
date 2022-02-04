// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveForward extends CommandBase {
  public static final double driveDistance = 3;
  public static double startingDistance;

  // PID Constansts
  private static final double kP = .125;
  private static final double kI = .02;
  private static final double kD = .125;
  
  private final PIDController m_pidController = new PIDController(kP, kI, kD);

  /** Creates a new DriveForward. */
  public DriveForward() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingDistance = Robot.m_robotDrive.getAverageDistance();
    SmartDashboard.putNumber("Target distance: ", startingDistance + driveDistance);

    m_pidController.setSetpoint(startingDistance + driveDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidOutput = m_pidController.calculate(Robot.m_robotDrive.getAverageDistance());
    Robot.m_robotDrive.m_drive(pidOutput, 0);
    SmartDashboard.putNumber("PID Output: ", pidOutput);

    //Robot.m_robotDrive.m_drive(.5, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.m_robotDrive.getAverageDistance() > (startingDistance + driveDistance)) {
      return true;
    } else {
      return false;
    }
  }

}
