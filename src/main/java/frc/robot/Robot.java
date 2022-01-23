/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Vector;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Drivetrain;
import frc.robot.globalDriveState;
import frc.robot.TestDriveFunctions;
import frc.robot.subsystems.ShooterTest;

import frc.robot.commands.DifferentialDriveWithJoysticks;
//import frc.robot.subsystems.Feeder;
//import frc.robot.subsystems.Hopper;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.Spinner;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private static String gameData;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static Drivetrain drivetrain = new Drivetrain();

  public static ShooterTest shooterTest = new ShooterTest();

  public static DifferentialDriveWithJoysticks differentialDriveWithJoysticks = new DifferentialDriveWithJoysticks();

  //public static DifferentialDriveWithJoysticks joystickCommand;
  //public static Spinner spinner;
  // public static Intake intake;
  // public static Shooter shooter; 
  // public static Feeder feeder;
  // public static Hopper hopper;
  public static OI oi = new OI();

  public static TestDriveFunctions testFunctions = new TestDriveFunctions(5000);

  public static RobotContainer robotContainer = new RobotContainer(); 
  // private final I2C.Port i2cPort = I2C.Port.kOnboard;

  // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  // private final ColorMatch m_colorMatcher = new ColorMatch();

  // private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  // private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  // private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  // private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    Scheduler.getInstance();
    
 
    //testFunctions.run();

    robotContainer = new RobotContainer();
    //drivetrain = new Drivetrain();
    //spinner = new Spinner();
    // intake = new Intake();
    // shooter = new Shooter();
    // feeder = new Feeder();
    // hopper = new Hopper();
    //oi = new OI();

    drivetrain.resetSensors();
    // m_colorMatcher.addColorMatch(kBlueTarget);
    // m_colorMatcher.addColorMatch(kGreenTarget);
    // m_colorMatcher.addColorMatch(kRedTarget);
    // m_colorMatcher.addColorMatch(kYellowTarget);  
  }

  
  @Override
  public void robotPeriodic() {
    //CommandScheduler.getInstance().enable();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */

   @Override 
   public void teleopInit()
   {
   }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    //globalDriveState.update++;
    
    //this is a temporary solution, because the drive command isn't actually getting scheduled
      // double throttle = 1;
       //Robot.drivetrain.drive(-Robot.oi.getLeftStick().getY()*throttle, Robot.oi.getRightStick().getX());
      

    //  Robot.drivetrain.drive(Robot.oi.getR2(), Robot.oi.getL2());
    //  Robot.drivetrain.reverse(Robot.oi.getL2(), Robot.oi.getR2());
    
    SmartDashboard.putNumber("driving status: ", testFunctions.timesRan);
    SmartDashboard.putNumber("Left Ticks: ", drivetrain.getLeftRawEncoderTicks());
    SmartDashboard.putNumber("Right Ticks: ", drivetrain.getRightRawEncoderTicks());
    SmartDashboard.putNumber("Right Distance: ", drivetrain.getRightDistance());
    SmartDashboard.putNumber("Left Distance: ", drivetrain.getLeftDistance());
    SmartDashboard.putNumber("Drive Distance: ", drivetrain.getDistance());
    SmartDashboard.putNumber("L2 Value", oi.getL2());
    

    // SmartDashboard.putBoolean("Break Beam:", feeder.getBreakBeam());
    //SmartDashboard.putBoolean("Right encoder  out of phase:", value);

    // SmartDashboard.putNumber("Red: ", spinner.getRed());
    // SmartDashboard.putNumber("Blue: ", spinner.getBlue());
    // SmartDashboard.putNumber("Green: ", spinner.getGreen());
    // SmartDashboard.putString("Color: ", spinner.getColor());

    gameData = DriverStation.getGameSpecificMessage();
  }

  public static String getGameData() {
    String color = ""; 
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B' :
          color = "Blue";
          break;
        case 'G' :
          color = "Green";
          break;
        case 'R' :
          color = "Red";
          break;
        case 'Y' :
          color = "Yellow";
          break;
        default :
          //This is corrupt data
          break;
      }
    } else {
      color = "Red";
    }
    return color;
  }

  /**
   * This function is called periodically during test mode.
   */
  // @Override
  // public void testPeriodic() {
  // }
  
}
