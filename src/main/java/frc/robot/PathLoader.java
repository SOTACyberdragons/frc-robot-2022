package frc.robot;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;

public class PathLoader {
    public static Trajectory 
        TEST_1, TEST_2, TEST_3, TEST_4,
        RIGHT_1, RIGHT_2, RIGHT_3, RIGHT_4;

    // Must set the reverse flags here, if applicable
    public static void initRightPaths() {
        RIGHT_1 = PathPlanner.loadPath("blueRight-1", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared);
        RIGHT_2 = PathPlanner.loadPath("blueRight-2", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared, true);
        RIGHT_3 = PathPlanner.loadPath("blueRight-3", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared);
        RIGHT_4 = PathPlanner.loadPath("blueRight-4", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared, true);
    }

    // Must set the reverse flags here, if applicable
    public static void initTestPaths() {
        TEST_1 = PathPlanner.loadPath("test-1", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared);
        TEST_2 = PathPlanner.loadPath("test-2", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared, true);
        TEST_3 = PathPlanner.loadPath("test-3", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared);
        TEST_4 = PathPlanner.loadPath("test-4", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared, true);
    }

//     public static void initRamsetes() {
//         // TODO This is the wrong pattern

//         // Test paths
//         T1ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, T1);
//         T2ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, T2);
//         T3ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, T3);
//         T4ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, T4);
        
//         // Right paths
//         R1ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, R1);
//         R2ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, R2);
//         R3ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, R3);
//         R4ram = new DrivetrainRamseteCommand(RobotContainer.m_drive, R4);
//     }
}
