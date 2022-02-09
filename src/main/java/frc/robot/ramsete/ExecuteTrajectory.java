package frc.robot.ramsete;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ExecuteTrajectory extends CommandBase {
    private boolean done = false;

    private Trajectory customTrajectory;

    public ExecuteTrajectory(Trajectory trajectory)
    {
        customTrajectory = trajectory;
    }

    public Command pathCommand() {
        RobotContainer.m_robotDrive.zeroHeading();
        RobotContainer.m_robotDrive.resetEncoders();

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

        var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
        var leftMeasurement = table.getEntry("left_measurement");
        var rightMeasurement = table.getEntry("right_measurement");

        var leftController = new PIDController(Constants.kPDriveVel, 0.25, .1);
        var rightController = new PIDController(Constants.kPDriveVel, 0.25, .1);

        RamseteCommand ramseteCommand = new RamseteCommand(
                this.customTrajectory,
                RobotContainer.m_robotDrive::getPose,
                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        Constants.ksVolts,
                        Constants.kvVoltSecondsPerMeter,
                        Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                RobotContainer.m_robotDrive::getWheelSpeeds,
                leftController,
                rightController,
                // RamseteCommand passes volts to the callback
                (leftVolts, rightVolts) -> {
                    RobotContainer.m_robotDrive.tankDriveVolts(leftVolts, rightVolts);
                    leftMeasurement.setNumber(leftVolts);
                    rightMeasurement.setNumber(rightVolts);
                },
                RobotContainer.m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        RobotContainer.m_robotDrive.resetOdometry(this.customTrajectory.getInitialPose());

        // Run path following command, then stop at the end.

        ramseteCommand.andThen(() -> this.finish());

        return ramseteCommand;
    }

    public void execute()
    {
        pathCommand().execute();
    }

    public void finish()
    {
        RobotContainer.m_robotDrive.tankDriveVolts(0, 0);
        this.done = true;
    }

    public boolean isFinished()
    {
        return this.done;
    }
}
