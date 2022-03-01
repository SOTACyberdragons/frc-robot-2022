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

package frc.robot.broken_stuff;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShootCargo extends CommandBase {
    public static double motorRPM;
    public static double motorFF;

    private double power;

    /** Creates a new ShootWithFalcon. */
    public ShootCargo(double targetRPM, double targetFF, double power) {
        motorRPM = targetRPM;
        motorFF = targetFF;
        addRequirements(Robot.m_shooter, Robot.m_feeder);

        this.power = power;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //Robot.m_shooter.setVelocity(motorRPM, motorFF);

        Robot.m_shooter.setPower(power);

        try {
            Thread.sleep(700);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (Robot.m_shooter.getVelocity() < (motorRPM *.95)) {
        //     Robot.m_feeder.feederStop();
        // } else {
        //     Robot.m_feeder.feederIn();
        // }

        Robot.m_feeder.feederIn();
        
        // DPad Controls to tune the Falcon Shooter
        // int dPad = RobotContainer.m_controller.getPOV();
        // if (dPad == 0) {
        //     motorRPM = motorRPM + 100;
        //     Robot.m_shooter.setVelocity(motorRPM, motorFF);
        // } else if (dPad == 180) {
        //     motorRPM = motorRPM + 100;
        //     Robot.m_shooter.setVelocity(motorRPM, motorFF);
        // }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.m_shooter._rightMaster.set(TalonFXControlMode.PercentOutput, 0);
        Robot.m_shooter._leftMaster.set(TalonFXControlMode.PercentOutput, 0);
        Robot.m_feeder.feederStop();;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
