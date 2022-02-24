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

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CommandGroupExample;
import frc.robot.commands.DriveForward;
import frc.robot.commands.RamseteTest;
import frc.robot.commands.ShootCargo;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.TurnWithGyro;
import frc.robot.grip.ImageProcessor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

    public static Drivetrain m_robotDrive = new Drivetrain();
    public static Shooter m_shooter = new Shooter();

    public static ImageProcessor imgProcessor = new ImageProcessor();

    private static NetworkTable ballData = NetworkTableInstance.getDefault().getTable("Ball Data");

    public static NetworkTableEntry ballDistance = ballData.getEntry("Ball Distance");

    public static NetworkTableEntry ballCenter = ballData.getEntry("ball center");

    // Adding XBox Controller Supports
    public static XboxController m_controller = new XboxController(3);
    final JoystickButton buttonA = new JoystickButton(m_controller, 1);
    final JoystickButton buttonB = new JoystickButton(m_controller, 2);
    final JoystickButton buttonX = new JoystickButton(m_controller, 3);
    final JoystickButton buttonY = new JoystickButton(m_controller, 4);

    // Adding slew value for the XBox Controller joysticks
    public static SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    public static SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public static double motorPower = 0.5;

    // Return XBox left stick for throttle control
    public static double getXBoxThrottle() {
        // return -m_speedLimiter.calculate(m_controller.getLeftY()) *
        // Constants.kMaxSpeed;
        return -m_speedLimiter.calculate(m_controller.getLeftY() * Constants.kMaxDriveSpeed);
    }

    // Return XBox right stick for rotational control
    public static double getXBoxRotation() {
        // return -m_rotLimiter.calculate(m_controller.getRightX()) *
        // Constants.kMaxAngularSpeed;
        return m_rotLimiter.calculate(m_controller.getRightX() * Constants.kMaxTurnSpeed);
    }

    public static double getXBoxPOV() {
        return m_controller.getPOV();
    }

    private void configureButtonBindings() {
        // Test commands
        // buttonA.whenPressed(new DriveForward(3));
        // buttonB.whenPressed(new TurnWithGyro(-90)); // TODO Fix me!
        // buttonX.whenPressed(new RamseteTest());
        
        buttonA.whenHeld(new SpinIntake(.5));
        // This sets the shooter speed in RPM. Don't overdo it
        buttonY.whenHeld(new ShootCargo(100, .15));
    }

    public RobotContainer() {
        configureButtonBindings();
    }

    RamseteTest autonomousCommand = new RamseteTest();

    public Command getAutonomousCommand() {
        return new CommandGroupExample(this);
    }
}
