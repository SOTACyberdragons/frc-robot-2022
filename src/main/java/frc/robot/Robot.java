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

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DifferentialDriveWithJoysticks;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    public static RobotContainer m_robotContainer;
    public static Intake m_intake = new Intake();
    public static Feeder m_feeder = new Feeder();
    public static Shooter m_shooter = new Shooter();
    public static Climber m_climber = new Climber();

    // TODO Old REV sensor code. Doesn't work. Replace with PWF Sensor code
    // public static MultiplexedDistanceSensor m_leftSensor;
    // public static MultiplexedDistanceSensor m_rightSensor;

    @Override
    public void robotInit() {
        System.out.println("Robot initialized!");

        RobotContainer.m_drive.zeroHeading();
        RobotContainer.m_drive.resetEncoders();

        RobotContainer.m_drive.resetOdometry(RobotContainer.m_drive.getPose());

        m_robotContainer = new RobotContainer();

        CommandScheduler.getInstance();
        CommandScheduler.getInstance().enable();

        // Reset the Falcon encoders
        RobotContainer.m_drive.leftMaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        RobotContainer.m_drive.leftSlave.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        RobotContainer.m_drive.rightMaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        RobotContainer.m_drive.rightSlave.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);

        // initialize trajectories and ramsetes
        PathContainer.initBlueRightPaths();

        // Create entries allowing for realtime modification without a deploy
        // TODO Remove these before competition
        SmartDashboard.putNumber("High Target RPS", Constants.kShooterRPSHigh);
        SmartDashboard.putNumber("High Target Spin", Constants.kShooterFeederBackspinHigh);
        SmartDashboard.putNumber("Low Target RPS", Constants.kShooterRPSLow);
        SmartDashboard.putNumber("Low Target Spin", Constants.kShooterFeederBackspinLow);

        // TODO Old REV sensor code. Doesn't work. Replace with PWF Sensor code
        // m_leftSensor = new MultiplexedDistanceSensor(I2C.Port.kOnboard, 7);
        // m_rightSensor = new MultiplexedDistanceSensor(I2C.Port.kOnboard, 6);
        // m_leftSensor.setAutomaticMode(true);
        // m_rightSensor.setAutomaticMode(true);
        // m_leftSensor.setDistanceUnits(Unit.kMillimeters);
        // m_rightSensor.setDistanceUnits(Unit.kMillimeters);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // TODO Old REV sensor code. Doesn't work. Replace with PWF Sensor code
        // SmartDashboard.putNumber("Left Range", m_leftSensor.getRange());
        // SmartDashboard.putNumber("Right Range", m_rightSensor.getRange());
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().enable();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        System.out.println("Teleop initialized!");
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().enable();

        RobotContainer.m_drive.zeroHeading();
        RobotContainer.m_drive.resetEncoders();

        CommandScheduler.getInstance().setDefaultCommand(RobotContainer.m_drive,
                new DifferentialDriveWithJoysticks());
    }

    @Override
    public void teleopPeriodic() {
        // Definate input for realtime modification without a deploy
        // TODO Remove these before competition
        Constants.kShooterRPSHigh = SmartDashboard.getNumber("High Target RPS", Constants.kShooterRPSHigh);
        Constants.kShooterFeederBackspinHigh = SmartDashboard.getNumber("High Target Spin", Constants.kShooterFeederBackspinHigh);
        Constants.kShooterRPSLow = SmartDashboard.getNumber("Low Target RPS", Constants.kShooterRPSLow);
        Constants.kShooterFeederBackspinLow = SmartDashboard.getNumber("Low Target Spin", Constants.kShooterFeederBackspinLow);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().enable();
    }

    @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();
    }

}
