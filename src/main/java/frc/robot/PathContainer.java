package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.commands.AutoCommands.DrivetrainRamseteCommand;

public class PathContainer {
    public static Trajectory 
        T1, T2, T3, T4,
        R1, R2, R3, R4;

    public static DrivetrainRamseteCommand
        T1ram, T2ram, T3ram, T4ram, 
        R1ram, R2ram, R3ram, R4ram;

    // Must set the reverse flags here, if applicable
    public static void initRightPaths() {
        R1 = PathPlanner.loadPath("blueRight-1", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared);
        R2 = PathPlanner.loadPath("blueRight-2", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared, true);
        R3 = PathPlanner.loadPath("blueRight-3", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared);
        R4 = PathPlanner.loadPath("blueRight-4", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared, true);
    }

    // Must set the reverse flags here, if applicable
    public static void initTestPaths() {
        T1 = PathPlanner.loadPath("test-1", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared);
        T2 = PathPlanner.loadPath("test-2", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared, true);
        T3 = PathPlanner.loadPath("test-3", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared);
        T4 = PathPlanner.loadPath("test-4", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared, true);
    }

    public static void initRamsetes() {
        // TODO This is the wrong pattern

        // Test paths
        T1ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, T1);
        T2ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, T2);
        T3ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, T3);
        T4ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, T4);
        
        // Right paths
        R1ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, R1);
        R2ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, R2);
        R3ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, R3);
        R4ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, R4);
    }
}
