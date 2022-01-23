package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ShootTest;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShooterTest;
import frc.robot.utils.AutoConstants;
import frc.robot.utils.DriveConstants;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



public class RobotContainer {

    
    
    public static  Joystick leftStick = new Joystick(0);
    public static Joystick rightStick = new Joystick(1);

    public JoystickButton leftTrigger = new JoystickButton(leftStick, 1), leftButton2 = new JoystickButton(leftStick, 2),
            leftButton3 = new JoystickButton(leftStick, 3), leftButton4 = new JoystickButton(leftStick, 4),
            leftButton5 = new JoystickButton(leftStick, 5), leftButton6 = new JoystickButton(leftStick, 6),
            leftButton7 = new JoystickButton(leftStick, 7), leftButton8 = new JoystickButton(leftStick, 8),
            leftButton9 = new JoystickButton(leftStick, 9), leftButton10 = new JoystickButton(leftStick, 10),
            leftButton11 = new JoystickButton(leftStick, 11), leftButton12 = new JoystickButton(leftStick, 12);
    

    public double getLeftJoyX() {
        return leftStick.getRawAxis(0);
    }

    public static double getLeftJoyY() {
        return leftStick.getRawAxis(1);
    }

    public double getLeftJoyThrottle() { 
        return leftStick.getRawAxis(2);
    }

    public JoystickButton rightTrigger = new JoystickButton(rightStick, 1),
            rightButton2 = new JoystickButton(rightStick, 2), rightButton3 = new JoystickButton(rightStick, 3),
            rightButton4 = new JoystickButton(rightStick, 4), rightButton5 = new JoystickButton(rightStick, 5),
            rightButton6 = new JoystickButton(rightStick, 6), rightButton7 = new JoystickButton(rightStick, 7),
            rightButton8 = new JoystickButton(rightStick, 8), rightButton9 = new JoystickButton(rightStick, 9),
            rightButton10 = new JoystickButton(rightStick, 10), rightButton11 = new JoystickButton(rightStick, 11),
            rightButton12 = new JoystickButton(rightStick, 12);

    public static double getRightJoyX() {
        return rightStick.getRawAxis(0);
    }

    public double getRightJoyY() {
        return rightStick.getRawAxis(1);
    }

    public double getRightJoyThrottle() {
        return rightStick.getRawAxis(2);
    }

    public RobotContainer() {
        configureButtonBindings();
    
        
    }

    private void configureButtonBindings() {

        rightTrigger.whileHeld(new ShootTest());

    }
  public Command getAutonomousCommand() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    
    

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        Robot.m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        Robot.m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        Robot.m_robotDrive::tankDriveVolts,
        Robot.m_robotDrive
    );

    // Reset odometry to the starting pose of the trajectory.

    Robot.m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> Robot.m_robotDrive.tankDriveVolts(0, 0));
  }


    // Run path following command, then stop at the end.

    //return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

}


