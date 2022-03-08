//                                                 @                             
//                                                  &@@                           
//                          * .                    * @@@                          
//                           * (@   ,                 @@@@                        
//                               @@@*       /          @@@@                       
//                                @@@@@@    @@(     ,* ,@@@@@                     
//                         %@@@@/*  @@@@@@@@       ,**. @@@@@@                    
//                      #********,    @@@@@@@@@@    ***  @@@@@@                   
//                   **********    /    @@@@@@@@@@@@   ,  @@@@@@                  
//                              &@@/  (@  (@@@@@@@@@@@@   @@@@@@@                 
//                            @@@@@//  @@@@@@@@@@@@@@@@@@& @@@@@@@                
//                          @@@@@@@//  @@@@@@@@# .@@@@@@@@@@@@@@@@                
//                         @@@@@@&///  %@@@@@@@@(  *  @@@@@@@@@@@@                
//                       *@@@@@//   @@@@@@@@@@@@@@%     @@@@@@@@@@@               
//                      .@@@@@@@@@@//   .@@@@@@@@@@@@@@  @@@@@@@@@@@              
//                      @@@@@@@@@@@@@@(/     @@@@@@@@@@@@@@@@@@@@@@@@@            
//                   @ %@@@@@@@@@@@@@@   ,  @@@@@@@@@@@@@@@@@@@@@@@@@@@           
//                  @@ @@@@@@@@@@@@@   .             *@@@@@@@@@  @@@@@@#          
//                 @@@ @@@@@@@@@@@@%   *******@@@&///     &@@@@@@@@@@@@@          
//                 @**  @@@@@@@@@@@   ******@@@@@@,          @@@@@@@@@@           
//                 #*** @@@@@@@@@@@   *****@@@@@                  @@@@*           
//                ***   @@@@@@@@@@@  ,****@@@,                                    
//                 *      @@@@@@@@@@.  *****@@                                    
//                          @@@@@@@@@#   ***%@                                    
//                           ,@@@@@@@@@    ***@,  /                               
//                              @@@@@@@@@(    ***   //////*.     */               
//                                 //@@@@@@%/      *    ///////                   
//                                 @    //////////                                
//                                   @@**                                         
//                                       @*****                                   
//                                             *                                  

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SpinOutake extends CommandBase {
  public static double kIntakeSpeed = -0.75;
  public static double kFeederSpeed = -0.5;

  /** Creates a new Outake. */
  public SpinOutake() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() { 
      Robot.m_intake.startIntake(kIntakeSpeed);        
      Robot.m_feeder.feederIn(kFeederSpeed);
      
      // Left rumble is always feeder
      RobotContainer.m_controller.setRumble(RumbleType.kLeftRumble, kFeederSpeed);
      RobotContainer.m_controller.setRumble(RumbleType.kRightRumble, kIntakeSpeed);
  } 

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {        
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
      return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
      Robot.m_intake.stopIntake();
      Robot.m_feeder.feederStop();

      RobotContainer.m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
      RobotContainer.m_controller.setRumble(RumbleType.kRightRumble, 0.0);
  }
}
