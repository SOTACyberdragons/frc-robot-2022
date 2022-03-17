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

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootCargo;
import frc.robot.commands.SpinIntake;

public class AutoRight extends SequentialCommandGroup {

    private static final Trajectory RIGHT_1 = PathPlanner.loadPath("right-1", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared);
    private static final Trajectory RIGHT_2 = PathPlanner.loadPath("right-2", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared, true);
    private static final Trajectory RIGHT_3 = PathPlanner.loadPath("right-3", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared);
    private static final Trajectory RIGHT_4 = PathPlanner.loadPath("right-4", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared, true);

    public AutoRight() {

        // See if changing paths to Robot relative works better
        // RobotContainer.m_drive.resetOdometry(RIGHT_1.getInitialPose());

        addCommands(
                new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, RIGHT_1).robotRelative(),
                        new SpinIntake()),
                new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, RIGHT_2),
                        new SpinUpShooter("Low")),
                new ParallelDeadlineGroup(new WaitCommand(1), new ShootCargo("Low")),
                new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, RIGHT_3),
                        new SpinIntake()),
                new ParallelDeadlineGroup(new WaitCommand(1), new SpinIntake()),
                new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, RIGHT_4),
                        new SpinUpShooter("Low")),
                new ParallelDeadlineGroup(new WaitCommand(2), new ShootCargo("Low")));
    }
}