package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TestTurnToAngle extends PIDCommand {
    // Motor characterization
    private static double kS = 1.0048;
    private static double kV = 6.0896;
    private static double kA = 0.17093;

    // Motor PID values
    private static double kP = 0.075749;
    private static double kI = 0;
    private static double kD = 0.0025432;

    // Constraints
    private static double kTurnToleranceDeg = 5;
    private static double kTurnRateToleranceDegPerS = 10; // degrees per second

    double angleTarget;

    private static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    /* Turn the robot to the specified angle. */
    public TestTurnToAngle(double targetAngleDegrees, Drivetrain m_drive) {
        /**
         * Turns to robot to the specified angle.
         *
         * @param targetAngleDegrees The angle to turn to
         * @param drive              The drive subsystem to use
         */
        super(
                // The controller that the command will use
                new PIDController(kP, kI, kD),
                // Close loop on heading
                m_drive::getHeadingDouble,
                // Set reference to target
                targetAngleDegrees,
                // Pipe output to turn robot
                output -> m_drive.arcadeDrive(0,
                        MathUtil.clamp(output + feedforward.calculate(targetAngleDegrees), -0.5, 0.5) * -1),
                // Require the drive
                m_drive);

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);

        // Set the controller tolerance
        getController().setTolerance(kTurnToleranceDeg, kTurnRateToleranceDegPerS);
    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        System.out.println("Current Heading " + RobotContainer.m_drive.getHeadingDouble());
        System.out.println("Current Target " + angleTarget);

        return getController().atSetpoint();
    }
}
