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

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.*;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RamseteTest extends RamseteCommand {
    public RamseteTest() {
        super(trajectory(), RobotContainer.m_drive::getPose,
                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        Constants.ksVolts,
                        Constants.kvVoltSecondsPerMeter,
                        Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                RobotContainer.m_drive::getWheelSpeeds,
                new PIDController(Constants.kPDriveVel, 0, 0),
                new PIDController(Constants.kPDriveVel, 0, 0),
                RobotContainer.m_drive::tankDriveVolts,
                RobotContainer.m_drive);

        this.andThen(() -> RobotContainer.m_drive.tankDriveVolts(0, 0));
    }

    public static Trajectory trajectory() {
        Trajectory trajectory;

        TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        Constants.ksVolts,
                        Constants.kvVoltSecondsPerMeter,
                        Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics, 10);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.kDriveKinematics)
                        .addConstraint(autoVoltageConstraint);

        trajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(0)),
                // Pass config
                config);

        return trajectory;
    }
}
