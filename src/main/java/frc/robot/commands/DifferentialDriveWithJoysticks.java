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

import static frc.robot.subsystems.TensorVision.m_targets;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TensorVision;

/** An example command that uses an example subsystem. */
public class DifferentialDriveWithJoysticks extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    TensorVision m_TensorVision = new TensorVision();
    PIDController turnController = new PIDController(Constants.kPAngular, Constants.kIAngular, Constants.kDAngular);

    public DifferentialDriveWithJoysticks() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double forwardSpeed;
        double rotationSpeed;
        double targetYaw;

        forwardSpeed = RobotContainer.getXBoxThrottle();
        rotationSpeed = RobotContainer.getXBoxRotation();

        if (RobotContainer.m_sensors.getDistanceOffset() < (Constants.kShooterSweetSpot * 1.5)) {
            RobotContainer.m_controller.setRumble(RumbleType.kRightRumble, 0.5);
        } else {
            RobotContainer.m_controller.setRumble(RumbleType.kRightRumble, 0);
        }

        // If we have a target, rumble the controller
        if (TensorVision.hasTargets(m_targets, RobotContainer.getTeamColor())) {
            RobotContainer.m_controller.setRumble(RumbleType.kLeftRumble,
                    TensorVision.getRumbleStrength(TensorVision.m_targets, RobotContainer.getTeamColor()));

            // If the B button is pressed, correct the angle
            if (RobotContainer.m_controller.getBButton()) {
                // Calculate angular turn power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw

                // Poor man's PID
                targetYaw = TensorVision.getTargetYaw(TensorVision.m_targets, RobotContainer.getTeamColor());

                // TODO Should slightly turn towards target. Tune these values if needed
                if (targetYaw > 2) {
                    rotationSpeed = -0.15;
                } else if (targetYaw < 2) {
                    rotationSpeed = 0.15;
                }
                // rotationSpeed = -turnController.calculate(TensorVision.getTargetYaw(TensorVision.m_targets, RobotContainer.getTeamColor()), 0);
            }
        } else {
            RobotContainer.m_controller.setRumble(RumbleType.kLeftRumble, 0);
        }
        RobotContainer.m_drive.m_drive(forwardSpeed, rotationSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
