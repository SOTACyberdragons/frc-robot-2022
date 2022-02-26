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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DifferentialDriveWithJoysticks;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TalonExample;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    public static RobotContainer m_robotContainer;
    public static Intake m_intake = new Intake();
    public static Feeder m_feeder = new Feeder();
    public static Shooter m_shooter = new Shooter();
    public static TalonExample m_shooterTest = new TalonExample();

    // TODO Old REV sensor code. Doesn't work. Replace with PWF Sensor code
    // public static MultiplexedDistanceSensor m_leftSensor;
    // public static MultiplexedDistanceSensor m_rightSensor;

    @Override
    public void robotInit() {
        RobotContainer.m_robotDrive.zeroHeading();
        RobotContainer.m_robotDrive.resetEncoders();

        RobotContainer.m_robotDrive.resetOdometry(RobotContainer.m_robotDrive.getPose());

        m_robotContainer = new RobotContainer();

        CommandScheduler.getInstance();
        CommandScheduler.getInstance().enable();

        // Reset the Falcon encoders
        RobotContainer.m_robotDrive.leftMaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        RobotContainer.m_robotDrive.leftSlave.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        RobotContainer.m_robotDrive.rightMaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        RobotContainer.m_robotDrive.rightSlave.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);

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
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().enable();

        RobotContainer.m_robotDrive.zeroHeading();
        RobotContainer.m_robotDrive.resetEncoders();

        CommandScheduler.getInstance().setDefaultCommand(RobotContainer.m_robotDrive,
                new DifferentialDriveWithJoysticks());
    }

    @Override
    public void teleopPeriodic() {

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
