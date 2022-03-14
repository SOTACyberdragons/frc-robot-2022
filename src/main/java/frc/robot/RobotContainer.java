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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ClimberArms;
import frc.robot.commands.ClimberPivot;
import frc.robot.commands.PostUp;
import frc.robot.commands.ShootCargo;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinOutake;
import frc.robot.commands.AutoCommands.AutoRight;
import frc.robot.commands.AutoCommands.AutoTest;
import frc.robot.commands.AutoCommands.AutoLeft;
import frc.robot.commands.AutoCommands.AutoMiddle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SensorArray;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.JoystickAnalogButton;

public class RobotContainer {

    // Subsystems
    public final static Drivetrain m_drive = new Drivetrain();
    public final static Shooter m_shooter = new Shooter();
    public final static Intake m_intake = new Intake();
    public final static Feeder m_feeder = new Feeder();
    public final static Climber m_climber = new Climber();
    public final static SensorArray m_sensors = new SensorArray();

    // Autonomous
    private static SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Team color
    private static SendableChooser<String> colorChooser = new SendableChooser<>();

    // Adding XBox Controller Supports
    public final static XboxController m_controller = new XboxController(0);
    private final static JoystickButton buttonA = new JoystickButton(m_controller, 1);
    private final static JoystickButton buttonB = new JoystickButton(m_controller, 2);
    private final static JoystickButton buttonX = new JoystickButton(m_controller, 3);
    private final static JoystickButton buttonY = new JoystickButton(m_controller, 4);
    private final static JoystickButton bumperL = new JoystickButton(m_controller, 5);
    private final static JoystickButton bumperR = new JoystickButton(m_controller, 6);
    private final static JoystickButton leftStick = new JoystickButton(m_controller, 9);
    private final static JoystickButton rightStick = new JoystickButton(m_controller, 10);

    // Turn the triggers into buttons
    private final static JoystickAnalogButton triggerR = new JoystickAnalogButton(m_controller, 3, 0.5);
    private final static JoystickAnalogButton triggerL = new JoystickAnalogButton(m_controller, 2, 0.5);

    // Turn the D_Pad into buttons
    private final static POVButton dPadUp = new POVButton(m_controller, 0);
    private final static POVButton dPadDown = new POVButton(m_controller, 180);

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

    private void configureButtonBindings() {
        // Intake buttons
        triggerR.whenHeld(new SpinIntake());
        bumperR.whenHeld(new SpinOutake());

        // Shooter buttons
        triggerL.whenHeld(new ShootCargo("High"));
        bumperL.whenHeld(new ShootCargo("Low"));

        // Targeting buttons
        buttonX.whenHeld(new PostUp());

        // Climber arm buttons
        rightStick.whenHeld(new ClimberArms("UP"));
        leftStick.whenHeld(new ClimberArms("DOWN"));

        dPadUp.whenHeld(new ClimberPivot("FORWARD"));
        dPadDown.whenHeld(new ClimberPivot("BACKWARD"));
    }

    public RobotContainer() {
        LiveWindow.disableAllTelemetry();
        configureButtonBindings();
        configureAuto();
        configureColor();
    }

    public void configureAuto() {
        autoChooser.addOption("Test Path", new AutoTest(this));
        autoChooser.setDefaultOption("Right (4 Ball)", new AutoRight(this));
        autoChooser.addOption("Left (2 Ball Popcorn)", new AutoLeft(this));
        autoChooser.addOption("Middle (2 Ball Popcorn)", new AutoMiddle(this));
        SmartDashboard.putData("Autonomous", autoChooser);
    }

    public void configureColor() {
        colorChooser.addOption("Red", "red");
        colorChooser.setDefaultOption("Blue", "blue");
        SmartDashboard.putData("Team Color", colorChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static String getTeamColor() {
        return colorChooser.getSelected();
    }
}
