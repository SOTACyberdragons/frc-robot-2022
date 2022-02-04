package frc.robot.subsystems;

import java.util.ArrayList;


import com.ctre.phoenix.motorcontrol.ControlMode;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.DriveConstants;

public class SpinWithGyro extends CommandBase {
    void zeroGyro()
    {
        Robot.m_robotDrive.m_gyro.addYaw(-Robot.m_robotDrive.m_gyro.getYaw());
    }

    public SpinWithGyro()
    {

    }

    public Command spinWithGyroCommand()
    {
        // zeroGyro();

        // while (Robot.m_robotDrive.gyro.getYaw() < 181) {
        //     Robot.m_robotDrive.tankDriveVolts(0.5, -05);
        // }

        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
        Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter,
        Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        10);
        
        // Create config for trajectory
        TrajectoryConfig config =
        new TrajectoryConfig(
        Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);
        
        Trajectory trajectory;
        
        ArrayList<Translation2d> points = new ArrayList<Translation2d>();
        points.add(new Translation2d(1, 1));
        points.add(new Translation2d(2, -1));

        trajectory =
        TrajectoryGenerator.generateTrajectory (
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        points, 
        // End 10 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass configs
        config);
         
        RamseteCommand ramseteCommand =
        new RamseteCommand(
        trajectory,
        Robot.m_robotDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
        Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter,
        Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        Robot.m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        Robot.m_robotDrive::tankDriveVolts,
        Robot.m_robotDrive);
        
        // Reset odometry to the starting pose of the trajectory.
        Robot.m_robotDrive.resetOdometry(trajectory.getInitialPose());

        return ramseteCommand.andThen(() -> Robot.m_robotDrive.tankDriveVolts(0, 0));
    }

    // public void stop()
    // {
    //     Robot.m_robotDrive.leftSlave.set(ControlMode.PercentOutput, 0);
    //     Robot.m_robotDrive.rightSlave.set(ControlMode.PercentOutput, 0);
    // }
}
