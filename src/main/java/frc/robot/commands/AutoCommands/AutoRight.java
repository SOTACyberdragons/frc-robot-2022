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

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootCargo;
import frc.robot.commands.SpinIntake;
import frc.robot.utils.PathLoader;

public class AutoRight extends SequentialCommandGroup {
    public AutoRight(RobotContainer robot) {

        // TODO See if changing paths to Robot relative works better
        RobotContainer.m_drive.resetOdometry(PathLoader.RIGHT_1.getInitialPose());

        addCommands(
                new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, PathLoader.RIGHT_1),
                        new SpinIntake()),
                new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, PathLoader.RIGHT_2),
                        new SpinUpShooter("High")),
                new ParallelDeadlineGroup(new WaitCommand(1), new ShootCargo("High")),
                new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, PathLoader.RIGHT_3),
                        new SpinIntake()),
                new ParallelDeadlineGroup(new WaitCommand(1), new SpinIntake()),
                new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, PathLoader.RIGHT_4),
                        new SpinUpShooter("Low")),
                new ParallelDeadlineGroup(new WaitCommand(2), new ShootCargo("High")));
    }
}