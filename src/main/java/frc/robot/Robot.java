/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

//import frc.robot.subsystems.Feeder;
//import frc.robot.subsystems.Hopper;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.Spinner;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Command m_autonomousCommand;

  public static TestDriveFunctions testFunctions = new TestDriveFunctions(5000);
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    CommandScheduler.getInstance().enable();
    
    robotContainer = new RobotContainer();
  
    
  }

  
  @Override
  public void robotPeriodic() {
    //CommandScheduler.getInstance().enable();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();

    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */

   @Override 
   public void teleopInit()
   {
     if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
    }
   }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    //globalDriveState.update++;
  
    
    SmartDashboard.putNumber("driving status: ", testFunctions.timesRan);
    SmartDashboard.putNumber("Left Ticks: ", robotContainer.m_robotDrive.getLeftRawEncoderTicks());
    SmartDashboard.putNumber("Right Ticks: ", robotContainer.m_robotDrive.getRightRawEncoderTicks());
    SmartDashboard.putNumber("Right Distance: ", robotContainer.m_robotDrive.getRightDistance());
    SmartDashboard.putNumber("Left Distance: ", robotContainer.m_robotDrive.getLeftDistance());
    SmartDashboard.putNumber("Drive Distance: ", robotContainer.m_robotDrive.getDistance());
    SmartDashboard.putNumber("L2 Value", robotContainer.m_oi.getL2());
  

  
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
  
}
