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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommandGroup;
import frc.robot.commands.NewTurn;
import frc.robot.commands.RamseteTest;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.JoystickAnalogButton;

public class RobotContainer {

    public static Drivetrain m_drive = new Drivetrain();
    public static Shooter m_shooter = new Shooter();

    // Adding XBox Controller Supports
    public static XboxController m_controller = new XboxController(0);
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

    private void configureButtonBindings() {
        // Test commands
        // buttonA.whenPressed(new DriveForward(3));
        buttonX.whenPressed(new NewTurn(45));
        buttonB.whenPressed(new NewTurn(-45));

        // Spins the Intake and feeder. WARNING! Breakbeam behaviour doesn't work in
        // sunlight.
        triggerR.whenHeld(new SpinIntake());

        // This sets the shooter speed in RPS and feeder from 0 to 1.
        triggerL.whenHeld(new SpinShooter(50, 0.5));
        bumperL.whenHeld(new SpinShooter(25, 0.5));

        // buttonX.whenPressed(new TestTurnToAngle(90, m_drive).withTimeout(5));
        // buttonB.whenPressed(new TestTurnToAngleProfiled(-90,
        // m_drive).withTimeout(5));
    }

    public RobotContainer() {
        configureButtonBindings();
    }

    RamseteTest autonomousCommand = new RamseteTest();

    public Command getAutonomousCommand() {
        return new AutoCommandGroup(this);
    }
}
