package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

    double angleTarget;

    private static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    public TestTurnToAngle(double targetAngleDegrees, Drivetrain m_drive) {
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

        this.angleTarget = targetAngleDegrees;
    }

    @Override
    public void end(boolean interupted) {
        RobotContainer.m_robotDrive.tankDriveVolts(0, 0);
    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        System.out.println("Current Heading " + RobotContainer.m_robotDrive.getHeadingDouble());
        System.out.println("Current Target " + angleTarget);
        return getController().atSetpoint();
    }
}
