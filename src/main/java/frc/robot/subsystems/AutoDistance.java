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
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.AutoConstants;
import frc.robot.utils.DriveConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class AutoDistance extends SubsystemBase {
    

    // public void driveOnPath()
    // {
    //     var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             Constants.ksVolts,
    //             Constants.kvVoltSecondsPerMeter,
    //             Constants.kaVoltSecondsSquaredPerMeter),
    //         Constants.kDriveKinematics,
    //         10);

    //         System.out.println("Creating config\n");

    // // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //             Constants.kMaxSpeedMetersPerSecond,
    //             Constants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);

    //         Trajectory exampleTrajectory;

    //         System.out.println("defining trajectory");

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

    //             System.out.println("defining ramsete command");

    //     RamseteCommand ramseteCommand =
    //         new RamseteCommand(
    //             exampleTrajectory,
    //             Robot.m_robotDrive::getPose,
    //             new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    //             new SimpleMotorFeedforward(
    //                 Constants.ksVolts,
    //                 Constants.kvVoltSecondsPerMeter,
    //                 Constants.kaVoltSecondsSquaredPerMeter),
    //             Constants.kDriveKinematics,
    //             Robot.m_robotDrive::getWheelSpeeds,
    //             new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //             new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //             // RamseteCommand passes volts to the callback
    //             Robot.m_robotDrive::tankDriveVolts,
    //             Robot.m_robotDrive);

    //             // Reset odometry to the starting pose of the trajectory.
    //             Robot.m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());


    //             // Run path following command, then stop at the end.
    //             return ramseteCommand.andThen(() -> Robot.m_robotDrive.tankDriveVolts(0, 0));

    //         for (int i = 0; i <= 20; i++) {
    //             System.out.println("Executing autonomous command");
    //         }
    //     }

    
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
