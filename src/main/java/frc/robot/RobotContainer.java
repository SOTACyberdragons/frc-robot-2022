package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.SnakePath;
import frc.robot.subsystems.DrivetrainRefactored;


public class RobotContainer {
    public static DrivetrainRefactored m_robotDrive = new DrivetrainRefactored();

    // Adding XBox Controller Support
    public static XboxController m_controller = new XboxController(3);
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

    private void configureButtonBindings() {
        buttonA.whenPressed(new SnakePath());
    }

    public RobotContainer() {
        configureButtonBindings();
    }

    SnakePath autonomousCommand = new SnakePath();

    public Command getAutonomousCommand() {
        return autonomousCommand;
    }
}
