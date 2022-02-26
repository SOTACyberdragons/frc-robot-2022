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

/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
//import frc.robot.PathContainer;
import frc.robot.RobotContainer;

public class CommandGroupExample extends SequentialCommandGroup {

    Trajectory blueRight1 = PathPlanner.loadPath("blueRight-1", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared, true);
    Trajectory blueRight2 = PathPlanner.loadPath("blueRight-2", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared);
    Trajectory blueRight3 = PathPlanner.loadPath("blueRight-3", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared, false);
    Trajectory blueRight4 = PathPlanner.loadPath("blueRight-4", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared);
    Trajectory blueRight5 = PathPlanner.loadPath("blueRight-5", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared, true);

    public CommandGroupExample(RobotContainer robot) {
        // Starting up subsystems

        // RobotContainer.m_robotDrive.resetOdometry(PathContainer.path1.getInitialPose());

        RobotContainer.m_robotDrive.resetOdometry(blueRight1.getInitialPose());

        addCommands(
                new DrivetrainRamsete(RobotContainer.m_robotDrive, blueRight1),
                new WaitCommand(1),
                new DrivetrainRamsete(RobotContainer.m_robotDrive, blueRight2),
                new WaitCommand(1),
                new DrivetrainRamsete(RobotContainer.m_robotDrive, blueRight3));
                
    }
}