// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class TestTurnToAngleProfiled extends ProfiledPIDCommand {
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

  /** Creates a new TestTurnToAngleProfiled. */
  public TestTurnToAngleProfiled(double targetAngleDegrees, Drivetrain m_drive) {

    super(
        new ProfiledPIDController(
            kP,
            kI,
            kD,
            new TrapezoidProfile.Constraints(
                Constants.kMaxTurnSpeed,
                Constants.kMaxAccelerationMetersPerSecondSquared)),
        // Close loop on heading
        m_drive::getHeadingDouble,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        (output, setpoint) -> m_drive.arcadeDrive(0,
            MathUtil.clamp(output + feedforward.calculate(targetAngleDegrees), -0.5, 0.5) * -1),
        // Require the drive
        m_drive);

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(10, 10);

    this.angleTarget = targetAngleDegrees;

  }

  @Override
  public void end(boolean interupted) {
    RobotContainer.m_robotDrive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    System.out.println("Current Heading " + RobotContainer.m_robotDrive.getHeadingDouble());
    System.out.println("Current Target " + angleTarget);
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
