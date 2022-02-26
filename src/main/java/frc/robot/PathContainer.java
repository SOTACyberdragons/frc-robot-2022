package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DrivetrainRamsete;

public class PathContainer {
    public static WaitCommand wait = new WaitCommand(1);
    
    public static Trajectory path1, path2, path3, path4, path5;

    public static DrivetrainRamsete ram1, ram2, ram3, ram4, ram5;

    public static void initBlueRightPaths()
    {
        path1 = PathPlanner.loadPath("blueRight-2", Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared, true);

        path2 = PathPlanner.loadPath("blueRight-2", Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared, true);

        path3 = PathPlanner.loadPath("blueRight-3", Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared, true);

        path4 = PathPlanner.loadPath("blueRight-4", Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared, true);

        path5 = PathPlanner.loadPath("blueRight-5", Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared, true);

        generateRamsetes();
    }

    private static void generateRamsetes()
    {
        ram1 = new DrivetrainRamsete(RobotContainer.m_robotDrive, path1);
        ram2 = new DrivetrainRamsete(RobotContainer.m_robotDrive, path2);
        ram3 = new DrivetrainRamsete(RobotContainer.m_robotDrive, path3);
        ram4 = new DrivetrainRamsete(RobotContainer.m_robotDrive, path4);
        ram5 = new DrivetrainRamsete(RobotContainer.m_robotDrive, path5);
    }
}
