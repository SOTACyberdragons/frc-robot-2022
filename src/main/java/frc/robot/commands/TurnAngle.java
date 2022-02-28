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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurnAngle extends CommandBase {
    public static double rotationAmount;
    public static double startHeading;
    public static double targetHeading;

    // Motor characterization
    private static double kS = 1.0048;
    private static double kV = 6.0896;
    private static double kA = 0.17093;

    // Motor PID values
    private static double kP = 0.075749;
    private static double kI = 0;
    private static double kD = 0.0025432;

    // Trapezoid constraints 
    // TODO I suspect these are too low.
    private static double kMaxTurnSpeed = .5;
    private static double kMaxAccelerationMetersPerSecondSquared = .25;
    private static double kTurnTolerance = 2;

    private static TrapezoidProfile.Constraints m_profile = new TrapezoidProfile.Constraints(kMaxTurnSpeed, kMaxAccelerationMetersPerSecondSquared);
    private static ProfiledPIDController m_pidController = new ProfiledPIDController(kP, kI, kD, m_profile);
    private static SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA);

    /** Creates a new TurnWithGyro. */
    public TurnAngle(double angleInput) {
        // Use addRequirements() here to declare subsystem dependencies.
        rotationAmount = angleInput;
        addRequirements(RobotContainer.m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startHeading = RobotContainer.m_drive.getRotation();
        targetHeading = startHeading + rotationAmount;
        m_pidController.setGoal(targetHeading);

        // Tolerance is position, velocity
        m_pidController.setTolerance(kTurnTolerance);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double pidOutput = m_pidController.calculate(RobotContainer.m_drive.getRotation(), targetHeading) + feedForward.calculate(targetHeading);
        RobotContainer.m_drive.arcadeDrive(0, -pidOutput);
        RobotContainer.m_controller.setRumble(RumbleType.kRightRumble, 1 - pidOutput);
        RobotContainer.m_controller.setRumble(RumbleType.kLeftRumble, 0.25);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_drive.arcadeDrive(0, 0);
        RobotContainer.m_controller.setRumble(RumbleType.kRightRumble, 0);
        RobotContainer.m_controller.setRumble(RumbleType.kLeftRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_pidController.atSetpoint();
    }
}
