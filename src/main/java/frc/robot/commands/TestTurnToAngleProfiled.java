// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class TestTurnToAngleProfiled extends ProfiledPIDCommand {

    /*
     * TODO -- MOVE THIS STUFF INTO CONSTANTS AT SOME POINT!
     */

    // Motor characterization
    private static double kS = 1.0048;
    private static double kV = 6.0896;
    private static double kA = 0.17093;

    // Motor PID values
    private static double kP = 0.075749;
    private static double kI = 0;
    private static double kD = 0.0025432;

    // Constraints
    private static double kMaxTurnSpeed = .5;
    private static double kMaxAccelerationMetersPerSecondSquared = .25;

    private static double kTurnToleranceDeg = 5;
    private static double kTurnRateToleranceDegPerS = 10; // degrees per second

    double angleTarget;

    private static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    /* Turn the robot to the specified angle using a motion profile. */
    public TestTurnToAngleProfiled(double targetAngleDegrees, Drivetrain m_drive) {
        /**
         * Turns to robot to the specified angle using a motion profile.
         *
         * @param targetAngleDegrees The angle to turn to
         * @param drive              The drive subsystem to use
         */
        super(
                new ProfiledPIDController(kP, kI, kD,
                        new TrapezoidProfile.Constraints(
                                kMaxTurnSpeed,
                                kMaxAccelerationMetersPerSecondSquared)),
                // Close loop on heading
                m_drive::getHeadingDouble,
                // Set reference to target
                targetAngleDegrees,
                // Pipe output to turn robot
                (output, setpoint) -> m_drive.arcadeDrive(0, output + feedforward.calculate(targetAngleDegrees)),
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
