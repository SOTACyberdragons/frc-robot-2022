package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SpinIntake;

public class AutoSimple extends SequentialCommandGroup {
    private static final Trajectory LEFT_1 = PathPlanner.loadPath("left-1", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared);
    private static final Trajectory LEFT_2 = PathPlanner.loadPath("left-2", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared, true);
            
    public AutoSimple()
    {
        addCommands(
            new ParallelDeadlineGroup(new WaitCommand(1), new SpinUpShooter("High")),
            new ParallelDeadlineGroup(new WaitCommand(1), new Shoot("High")),
            new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_1).robotRelative(),
            new SpinIntake()),
            new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_1).robotRelative(),
            new SpinIntake()),
            new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_1).robotRelative(),
            new SpinIntake())
        //     new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_2),
        //     new ParallelDeadlineGroup(new WaitCommand(1), new SpinUpShooter("Low")),
        //     new ParallelDeadlineGroup(new WaitCommand(1), new Shoot("Low")),
        //     new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_1),
        //     new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_1)
            );
    }
}
