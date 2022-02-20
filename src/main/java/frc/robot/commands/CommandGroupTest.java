/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.PathPlanner;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CommandGroupTest extends SequentialCommandGroup {

    Trajectory blueRight1 = PathPlanner.loadPath("blueRight-1", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared, true);
    Trajectory blueRight2 = PathPlanner.loadPath("blueRight-2", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared);
    Trajectory blueRight3 = PathPlanner.loadPath("blueRight-3", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared, true);
    Trajectory blueRight4 = PathPlanner.loadPath("blueRight-4", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared);
    Trajectory blueRight5 = PathPlanner.loadPath("blueRight-5", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared, true);

    public CommandGroupTest(RobotContainer robot) {
        // Starting up subsystems

        RobotContainer.m_robotDrive.resetOdometry(blueRight1.getInitialPose());

        addCommands(
                new DrivetrainRamsete(RobotContainer.m_robotDrive, blueRight1),
                new WaitCommand(1),
                new DrivetrainRamsete(RobotContainer.m_robotDrive, blueRight2),
                new WaitCommand(1),
                new DrivetrainRamsete(RobotContainer.m_robotDrive, blueRight3),
                new WaitCommand(1),
                new DrivetrainRamsete(RobotContainer.m_robotDrive, blueRight4),
                new WaitCommand(1),
                new DrivetrainRamsete(RobotContainer.m_robotDrive, blueRight5));
    }
}