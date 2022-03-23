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
import frc.robot.commands.MoveIntake;
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
    public static final Drivetrain m_drive = new Drivetrain();
    public static final Shooter m_shooter = new Shooter();
    public static final Intake m_intake = new Intake();
    public static final Feeder m_feeder = new Feeder();
    public static final Climber m_climber = new Climber();
    public static final SensorArray m_sensors = new SensorArray();

    // Autonomous
    private static SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Team color
    private static SendableChooser<String> colorChooser = new SendableChooser<>();

    // Adding XBox Controller Supports
    public static final XboxController m_controller = new XboxController(0);
   // private final static JoystickButton buttonA = new JoystickButton(m_controller, 1);

    // Button B is indirectly mapped don't use this button.
    // private final static JoystickButton buttonB = new JoystickButton(m_controller, 2);

    private static final JoystickButton buttonX = new JoystickButton(m_controller, 3);
     private final static JoystickButton buttonY = new JoystickButton(m_controller, 4);
    private static final JoystickButton bumperL = new JoystickButton(m_controller, 5);
    private static final JoystickButton bumperR = new JoystickButton(m_controller, 6);
    private static final JoystickButton leftStick = new JoystickButton(m_controller, 9);
    private static final JoystickButton rightStick = new JoystickButton(m_controller, 10);

    // Turn the triggers into buttons
    private static final JoystickAnalogButton triggerR = new JoystickAnalogButton(m_controller, 3, 0.5);
    private static final JoystickAnalogButton triggerL = new JoystickAnalogButton(m_controller, 2, 0.5);

    // Turn the D_Pad into buttons
    private static final POVButton dPadUp = new POVButton(m_controller, 0);
    private static final POVButton dPadDown = new POVButton(m_controller, 180);

    // Adding slew value for the XBox Controller joysticks
    private static final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    private static final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

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

        //buttonA.whenPressed(new MoveIntake("up"));
        buttonY.whenPressed(new MoveIntake("down"));
    }

    public RobotContainer() {
        LiveWindow.disableAllTelemetry();
        configureButtonBindings();
        configureAuto();
        configureColor();
    }

    public void configureAuto() {
        autoChooser.addOption("Test Path", new AutoTest());
        autoChooser.setDefaultOption("Right (4 Ball)", new AutoRight());
        autoChooser.addOption("Left (2 Ball Popcorn)", new AutoLeft());
        autoChooser.addOption("Middle (2 Ball Popcorn)", new AutoMiddle());
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
