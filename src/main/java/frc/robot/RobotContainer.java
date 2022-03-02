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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCommandGroup;
import frc.robot.commands.RamseteTest;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinShooter;
import frc.robot.commands.TurnAngle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.JoystickAnalogButton;

public class RobotContainer {

    public static Drivetrain m_drive = new Drivetrain();
    public static Shooter m_shooter = new Shooter();

    // Adding XBox Controller Supports
    public static XboxController m_controller = new XboxController(3);
    final JoystickButton buttonA = new JoystickButton(m_controller, 1);
    final JoystickButton buttonB = new JoystickButton(m_controller, 2);
    final JoystickButton buttonX = new JoystickButton(m_controller, 3);
    final JoystickButton buttonY = new JoystickButton(m_controller, 4);
    final JoystickButton bumperL = new JoystickButton(m_controller, 5);

    public JoystickAnalogButton triggerR = new JoystickAnalogButton(m_controller, 3, 0.5);
    public JoystickAnalogButton triggerL = new JoystickAnalogButton(m_controller, 2, 0.5);

    // Adding slew value for the XBox Controller joysticks
    public static SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    public static SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    // Return XBox left stick for throttle control
    public static double getXBoxThrottle() {
        return -m_speedLimiter.calculate(m_controller.getLeftY() * Constants.kMaxDriveSpeed);
    }

    // Return XBox right stick for rotational control
    public static double getXBoxRotation() {
        return m_rotLimiter.calculate(m_controller.getRightX() * Constants.kMaxTurnSpeed);
    }

    public static double getXBoxPOV() {
        return m_controller.getPOV();
    }

    // Allows use to use the same command on multiple buttons
    // private static SpinShooter highShooter = new SpinShooter("High");
    // private static SpinShooter lowShooter = new SpinShooter("Low");

    private void configureButtonBindings() {
        // Test commands
        // buttonA.whenPressed(new DriveForward(3));
        buttonB.whenPressed(new TurnAngle(-30).withTimeout(2));

        // Spins the Intake and feeder. WARNING! Breakbeam behaviour doesn't work in
        // sunlight.
        triggerR.whenHeld(new SpinIntake());

        // Shooter buttons
        triggerL.whenHeld(new SpinShooter("High"));
        bumperL.whenHeld(new SpinShooter("Low"));
    }

    public RobotContainer() {
        configureButtonBindings();
    }

    RamseteTest autonomousCommand = new RamseteTest();

    public Command getAutonomousCommand() {
        return new AutoCommandGroup(this);
    }
}
