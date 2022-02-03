package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.ShootTest;

public class RobotContainer {

/*  Uncomment for Joystick Control   

    public static Joystick leftStick = new Joystick(0);
    public static Joystick rightStick = new Joystick(1);

    public JoystickButton leftTrigger = new JoystickButton(leftStick, 1),
            leftButton2 = new JoystickButton(leftStick, 2),
            leftButton3 = new JoystickButton(leftStick, 3),
            leftButton4 = new JoystickButton(leftStick, 4),
            leftButton5 = new JoystickButton(leftStick, 5),
            leftButton6 = new JoystickButton(leftStick, 6),
            leftButton7 = new JoystickButton(leftStick, 7),
            leftButton8 = new JoystickButton(leftStick, 8),
            leftButton9 = new JoystickButton(leftStick, 9),
            leftButton10 = new JoystickButton(leftStick, 10),
            leftButton11 = new JoystickButton(leftStick, 11),
            leftButton12 = new JoystickButton(leftStick, 12);

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
            rightButton2 = new JoystickButton(rightStick, 2),
            rightButton3 = new JoystickButton(rightStick, 3),
            rightButton4 = new JoystickButton(rightStick, 4),
            rightButton5 = new JoystickButton(rightStick, 5),
            rightButton6 = new JoystickButton(rightStick, 6),
            rightButton7 = new JoystickButton(rightStick, 7),
            rightButton8 = new JoystickButton(rightStick, 8),
            rightButton9 = new JoystickButton(rightStick, 9),
            rightButton10 = new JoystickButton(rightStick, 10),
            rightButton11 = new JoystickButton(rightStick, 11),
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
    */

    // Adding XBox Controller Support
    public static XboxController m_controller = new XboxController(0);
        final JoystickButton buttonA = new JoystickButton(m_controller, 1);

    // Adding slew value for the XBox Controller joysticks
    public static SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    public static SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    // Return XBox left stick for throttle control
    public static double getXBoxThrottle() {
        // return -m_speedLimiter.calculate(m_controller.getLeftY()) * Constants.kMaxSpeed;
        return -m_speedLimiter.calculate(m_controller.getLeftY() * Constants.kMaxDriveSpeed);
    }

    // Return XBox right stick for rotational control
    public static double getXBoxRotation() {
        // return -m_rotLimiter.calculate(m_controller.getRightX()) * Constants.kMaxAngularSpeed;
        return -m_rotLimiter.calculate(m_controller.getRightX() * Constants.kMaxTurnSpeed);
    }

    private void configureButtonBindings() {
        buttonA.whileHeld(new ShootTest());
        // rightButton3.whileHeld(new DriveAutoDistance());
    }

    public RobotContainer() {
        configureButtonBindings();
    }

}
