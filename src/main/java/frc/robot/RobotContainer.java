package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CommandGroupTest;
import frc.robot.commands.DriveForward;
import frc.robot.commands.ShootWithFalcon;
import frc.robot.commands.SnakePath;
import frc.robot.commands.TurnWithGyro;
import frc.robot.subsystems.DrivetrainRefactored;
import frc.robot.subsystems.FalconShooter;

public class RobotContainer {

    public static DrivetrainRefactored m_robotDrive = new DrivetrainRefactored();
    public static FalconShooter m_shooter = new FalconShooter();

    // Adding XBox Controller Support
    public static XboxController m_controller = new XboxController(3);
    final JoystickButton buttonA = new JoystickButton(m_controller, 1);
    final JoystickButton buttonB = new JoystickButton(m_controller, 2);
    final JoystickButton buttonX = new JoystickButton(m_controller, 3);
    final JoystickButton buttonY = new JoystickButton(m_controller, 4);

    // Adding slew value for the XBox Controller joysticks
    public static SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    public static SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    public static double motorPower = 0.5;

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
        return m_rotLimiter.calculate(m_controller.getRightX() * Constants.kMaxTurnSpeed);
    }

    public static double getXBoxPOV() {
        return m_controller.getPOV();
    }

    private void configureButtonBindings() {
        buttonA.whenPressed(new DriveForward(3));
        buttonB.whenPressed(new TurnWithGyro(-90));
        buttonX.whenPressed(new SnakePath());

        // This sets the shooter speed, max is 1
        buttonY.whenHeld(new ShootWithFalcon(.55));      // DPad Controls to tune the Falcon Shooter
        // double dPad = getXBoxPOV();

        // if (dPad == 0) { // DPAD UP button is pressed
        //     System.out.println("Up");
        //     if (motorPower + .1 < 1) {motorPower = motorPower + 0.1;};
        // } else if (dPad == 180) { // DPAD DOWN button is pressed
        //     if (motorPower - .1 > 0) {motorPower = motorPower - 0.1;};
        // }

    }

    public RobotContainer() {
        configureButtonBindings();
    }

    SnakePath autonomousCommand = new SnakePath();

    public Command getAutonomousCommand() {
        return new CommandGroupTest(this);
    }
}
