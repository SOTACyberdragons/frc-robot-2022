package frc.robot;

// For Copntrollers
import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Commands mapped to buttons
import frc.robot.commands.ShootTest;
import frc.robot.commands.DriveForward;
import frc.robot.commands.TurnWithGyro;
import frc.robot.subsystems.DrivetrainRefactored;

// For Ramsete functions
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import java.util.List;

public class RobotContainer {

    private final DrivetrainRefactored m_robotDrive = new DrivetrainRefactored();

    /*
     * Uncomment for Joystick Control
     * 
     * public static Joystick leftStick = new Joystick(0);
     * public static Joystick rightStick = new Joystick(1);
     * 
     * public JoystickButton leftTrigger = new JoystickButton(leftStick, 1),
     * leftButton2 = new JoystickButton(leftStick, 2),
     * leftButton3 = new JoystickButton(leftStick, 3),
     * leftButton4 = new JoystickButton(leftStick, 4),
     * leftButton5 = new JoystickButton(leftStick, 5),
     * leftButton6 = new JoystickButton(leftStick, 6),
     * leftButton7 = new JoystickButton(leftStick, 7),
     * leftButton8 = new JoystickButton(leftStick, 8),
     * leftButton9 = new JoystickButton(leftStick, 9),
     * leftButton10 = new JoystickButton(leftStick, 10),
     * leftButton11 = new JoystickButton(leftStick, 11),
     * leftButton12 = new JoystickButton(leftStick, 12);
     * 
     * public double getLeftJoyX() {
     * return leftStick.getRawAxis(0);
     * }
     * 
     * public static double getLeftJoyY() {
     * return leftStick.getRawAxis(1);
     * }
     * 
     * public double getLeftJoyThrottle() {
     * return leftStick.getRawAxis(2);
     * }
     * 
     * public JoystickButton rightTrigger = new JoystickButton(rightStick, 1),
     * rightButton2 = new JoystickButton(rightStick, 2),
     * rightButton3 = new JoystickButton(rightStick, 3),
     * rightButton4 = new JoystickButton(rightStick, 4),
     * rightButton5 = new JoystickButton(rightStick, 5),
     * rightButton6 = new JoystickButton(rightStick, 6),
     * rightButton7 = new JoystickButton(rightStick, 7),
     * rightButton8 = new JoystickButton(rightStick, 8),
     * rightButton9 = new JoystickButton(rightStick, 9),
     * rightButton10 = new JoystickButton(rightStick, 10),
     * rightButton11 = new JoystickButton(rightStick, 11),
     * rightButton12 = new JoystickButton(rightStick, 12);
     * 
     * public static double getRightJoyX() {
     * return rightStick.getRawAxis(0);
     * }
     * 
     * public double getRightJoyY() {
     * return rightStick.getRawAxis(1);
     * }
     * 
     * public double getRightJoyThrottle() {
     * return rightStick.getRawAxis(2);
     * }
     */

    // Adding XBox Controller Support
    public static XboxController m_controller = new XboxController(0);
    final JoystickButton buttonA = new JoystickButton(m_controller, 1);
    final JoystickButton buttonB = new JoystickButton(m_controller, 2);

    // Adding slew value for the XBox Controller joysticks
    public static SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    public static SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

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
        return -m_rotLimiter.calculate(m_controller.getRightX() * Constants.kMaxTurnSpeed);
    }

    // public static DrivetrainRefactored m_drive = new DrivetrainRefactored();

    private void configureButtonBindings() {
        // buttonA.whenPressed(new RamseteTest());
        buttonA.whenPressed(new DriveForward(3));
        buttonB.whenPressed(new TurnWithGyro(90));
    }

    public RobotContainer() {
        configureButtonBindings();
    }

    public Command getAutonomousCommand() {
        m_robotDrive.zeroHeading();
        m_robotDrive.resetEncoders();

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        Constants.ksVolts,
                        Constants.kvVoltSecondsPerMeter,
                        Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics, 10);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, 1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);

        RamseteCommand ramseteCommand = new RamseteCommand(
                exampleTrajectory,
                m_robotDrive::getPose,
                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        Constants.ksVolts,
                        Constants.kvVoltSecondsPerMeter,
                        Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.kPDriveVel, 0, 0),
                new PIDController(Constants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }
}
