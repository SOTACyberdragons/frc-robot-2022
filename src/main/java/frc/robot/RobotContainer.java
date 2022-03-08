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
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ShootCargo;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinOutake;
import frc.robot.commands.AutoCommands.AutoTest;
import frc.robot.commands.AutoCommands.AutoRight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.JoystickAnalogButton;

public class RobotContainer {

    // Subsystems
    public final static Drivetrain m_drive = new Drivetrain();
    public final static Shooter m_shooter = new Shooter();
    public final static Intake m_intake = new Intake();
    public final static Feeder m_feeder = new Feeder();
    public final static Climber m_climber = new Climber();

    // Autonomous
    private static SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Adding XBox Controller Supports
    public final static XboxController m_controller = new XboxController(0);
    private final static JoystickButton buttonA = new JoystickButton(m_controller, 1);
    private final static JoystickButton buttonB = new JoystickButton(m_controller, 2);
    private final static JoystickButton buttonX = new JoystickButton(m_controller, 3);
    private final static JoystickButton buttonY = new JoystickButton(m_controller, 4);
    private final static JoystickButton bumperL = new JoystickButton(m_controller, 5);
    private final static JoystickButton bumperR = new JoystickButton(m_controller, 6);

    // Turn the triggers into buttons 
    private final static JoystickAnalogButton triggerR = new JoystickAnalogButton(m_controller, 3, 0.5);
    private final static JoystickAnalogButton triggerL = new JoystickAnalogButton(m_controller, 2, 0.5);

    // Adding slew value for the XBox Controller joysticks
    private final static SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    private final static SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

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
        triggerR.whenHeld(new SpinIntake());
        bumperR.whenHeld(new SpinOutake());

        // Shooter buttons
        triggerL.whenHeld(new ShootCargo("High"));
        bumperL.whenHeld(new ShootCargo("Low"));
    }

    public RobotContainer() {
        // TODO Uncomment this for competition
        // LiveWindow.disableAllTelemetry();

        configureButtonBindings();
    }

    public void configureAuto() {
        // autoChooser.addOption("Do Nothing", new DoNothingAuton());
        autoChooser.addOption("Test", new AutoTest(this));
        autoChooser.setDefaultOption("Right", new AutoRight(this));
        SmartDashboard.putData("Autonomous", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
