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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class DriveForward extends CommandBase {
    public static double driveDistance;
    public static double startingDistance;
    public static double targetDistance;

    // PID Constansts
    private static final double kP = .5; // Power
    private static final double kI = .0075; // Ease in sensitivity
    private static final double kD = .125; // Smoothing
    private static final double kF = .2; // Feed forward

    public PIDController m_pidController = new PIDController(kP, kI, kD);

    /** Creates a new DriveForward. */
    public DriveForward(double distanceInMeters) {
        // Use addRequirements() here to declare subsystem dependencies.
        driveDistance = distanceInMeters;
        addRequirements(RobotContainer.m_robotDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startingDistance = RobotContainer.m_robotDrive.getAverageDistance();
        targetDistance = startingDistance + driveDistance;
        m_pidController.setSetpoint(targetDistance);
        m_pidController.setTolerance(.05, .05);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double pidOutput = MathUtil
                .clamp((m_pidController.calculate(RobotContainer.m_robotDrive.getAverageDistance()) + kF), -0.5, 0.5);
        RobotContainer.m_robotDrive.m_drive(pidOutput, 0);
        RobotContainer.m_controller.setRumble(RumbleType.kRightRumble,
                1 - (m_pidController.calculate(RobotContainer.m_robotDrive.getAverageDistance())));
        RobotContainer.m_controller.setRumble(RumbleType.kLeftRumble, 0.25);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_robotDrive.m_drive(0, 0);
        RobotContainer.m_controller.setRumble(RumbleType.kRightRumble, 0);
        RobotContainer.m_controller.setRumble(RumbleType.kLeftRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_pidController.atSetpoint();
    }
}
