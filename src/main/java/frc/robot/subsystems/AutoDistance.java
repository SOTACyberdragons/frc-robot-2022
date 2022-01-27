package frc.robot.subsystems;

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
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.AutoConstants;
import frc.robot.utils.DriveConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class AutoDistance extends SubsystemBase {
    

    public void driveOnPath()
    {
    //     DifferentialDriveVoltageConstraint autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             Constants.ksVolts,
    //             Constants.kvVoltSecondsPerMeter,
    //             Constants.kaVoltSecondsSquaredPerMeter),
    //         Constants.kDriveKinematics,
    //         10);

    // // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //             AutoConstants.kMaxSpeedMetersPerSecond,
    //             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.kDriveKinematics)
    //         // Apply the voltage constraint
    //         .addConstraint(autoVoltageConstraint);
    //         Trajectory exampleTrajectory;

    //         exampleTrajectory =
    //         TrajectoryGenerator.generateTrajectory(
    //             // Start at the origin facing the +X direction
    //             new Pose2d(0, 0, new Rotation2d(0)),
    //             // Pass through these two interior waypoints, making an 's' curve path
    //             List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //             // End 3 meters straight ahead of where we started, facing forward
    //             new Pose2d(3, 0, new Rotation2d(0)),
    //             // Pass config
    //             config);

    //     RamseteCommand ramseteCommand =
    //         new RamseteCommand(
    //             exampleTrajectory,
    //             Robot.m_robotDrive::getPose,
    //             new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    //             new SimpleMotorFeedforward(
    //                 DriveConstants.ksVolts,
    //                 DriveConstants.kvVoltSecondsPerMeter,
    //                 DriveConstants.kaVoltSecondsSquaredPerMeter),
    //             DriveConstants.kDriveKinematics,
    //             Robot.m_robotDrive::getWheelSpeeds,
    //             new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //             new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //             // RamseteCommand passes volts to the callback
    //             Robot.m_robotDrive::tankDriveVolts,
    //             Robot.m_robotDrive);
    }

    
    public void driveXDistance(double totalDistance)
    {
        double initialDistance = Robot.m_robotDrive.getDistance();

        while (true) {
            if (Robot.m_robotDrive.getDistance() - initialDistance >= totalDistance) {
                break;
            }
            
            Robot.m_robotDrive.arcadeDrive(0.5, 0);
        }
    }
}
