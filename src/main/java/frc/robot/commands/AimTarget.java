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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AimTarget extends CommandBase {

    // Calculation variables
    private double targetYaw;
    private double desiredYaw;
    private double errorYaw;

    // PID output variables
    private double rotationSpeed;

    PIDController turnController = new PIDController(Constants.kPAngular, Constants.kIAngular, Constants.kDAngular);

    /** Creates a new AimTarget. */
    public AimTarget(double turnAngle) {
        targetYaw = turnAngle;

        turnController.setSetpoint(0);
        turnController.setTolerance(2);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        desiredYaw = targetYaw + RobotContainer.m_drive.getRotation();
        // Set the initial PID error
        errorYaw = desiredYaw - RobotContainer.m_drive.getRotation();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // -1.0 required to ensure positive PID controller effort _increases_ ya
        rotationSpeed = -turnController.calculate(errorYaw, 0);

        RobotContainer.m_drive.arcadeDrive(0, rotationSpeed);
        // Update the PID error
        errorYaw = desiredYaw - RobotContainer.m_drive.getRotation();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_drive.arcadeDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return turnController.atSetpoint();
    }
}
