package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.commands.DrivetrainRamsete;

public class PathContainer {
    public static Trajectory bR1, bR2, bR3, bR4;

    public static DrivetrainRamsete bR1ram, bR2ram, bR3ram, bR4ram;

    public static void initBlueRightPaths() {
        bR1 = PathPlanner.loadPath("blueRight-1", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared);
        bR2 = PathPlanner.loadPath("blueRight-2", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared, true);
        bR3 = PathPlanner.loadPath("blueRight-3", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared);
        bR4 = PathPlanner.loadPath("blueRight-4", Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared, true);

        initRamsetes();
    }

    private static void initRamsetes() {
        bR1ram = new DrivetrainRamsete(RobotContainer.m_drive, bR1);
        bR2ram = new DrivetrainRamsete(RobotContainer.m_drive, bR2);
        bR3ram = new DrivetrainRamsete(RobotContainer.m_drive, bR3);
        bR4ram = new DrivetrainRamsete(RobotContainer.m_drive, bR4);
    }
}
