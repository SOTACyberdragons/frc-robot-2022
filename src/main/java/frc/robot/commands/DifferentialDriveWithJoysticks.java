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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TensorVision;

/** An example command that uses an example subsystem. */
public class DifferentialDriveWithJoysticks extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    // TODO THIS MUST BE A GLOBAL CONSTANT!
    private static String teamColor = "red";

    // Turn PID values scaled to 0 thru 1
    private final double kP = 0.025; // 0.1084;
    private final double kI = 0;
    private final double kD = 0.0003329; // 0.0039948;

    TensorVision m_TensorVision = new TensorVision();
    PIDController turnController = new PIDController(kP, kI, kD);

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

        forwardSpeed = RobotContainer.getXBoxThrottle();

        // If we have a target, rumble the controller
        if (TensorVision.hasTargets(TensorVision.m_targets, teamColor)) {
            RobotContainer.m_controller.setRumble(RumbleType.kLeftRumble,
                    TensorVision.getRumbleStrength(TensorVision.m_targets, teamColor));
        } else {
            RobotContainer.m_controller.setRumble(RumbleType.kLeftRumble, 0);
        }

        // If the 'A' button is pressed, aim towards target if one has been found
        if (RobotContainer.m_controller.getAButton()) {
            if (TensorVision.hasTargets(TensorVision.m_targets, teamColor)) {
                // Calculate angular turn power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                System.out.println("Target yaw: " + TensorVision.getTargetYaw(TensorVision.m_targets, teamColor));
                rotationSpeed = -turnController.calculate(TensorVision.getTargetYaw(TensorVision.m_targets, teamColor),
                        0);
                new SpinIntake();
            } else {
                // If we have no targets, don't turn.
                rotationSpeed = 0;
            }
        } else {
            // Manual Driver Mode
            rotationSpeed = RobotContainer.getXBoxRotation();
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
