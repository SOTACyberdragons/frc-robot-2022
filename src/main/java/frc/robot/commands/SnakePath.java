package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.*;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SnakePath extends RamseteCommand {
    public SnakePath()
    {
        super(trajectory(), RobotContainer.m_robotDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        RobotContainer.m_robotDrive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        RobotContainer.m_robotDrive::tankDriveVolts,
        RobotContainer.m_robotDrive);

        this.andThen(() -> RobotContainer.m_robotDrive.tankDriveVolts(0, 0));
    }       

    public static Trajectory trajectory()
    {
        Trajectory trajectory;

        TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        Constants.ksVolts,
                        Constants.kvVoltSecondsPerMeter,
                        Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics, 10);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.kDriveKinematics)
                        .addConstraint(autoVoltageConstraint);
        
        double robotX = RobotContainer.m_robotDrive.getPose().getX();
        double robotY = RobotContainer.m_robotDrive.getPose().getY();

        trajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(0)),
                // Pass config
                config);

                // new Pose2d(robotX, robotY, new Rotation2d(0)),
                // List.of(new Translation2d(robotX + 1, robotY + 1), new Translation2d(robotX + 2, robotY - 1)),
                // new Pose2d(robotX + 3, robotY, new Rotation2d(0)),
                // config);
        
        return trajectory;
    }
}
