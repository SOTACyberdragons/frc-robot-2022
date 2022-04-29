package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class MoveAcrossLine extends SequentialCommandGroup {
    private static final Trajectory LEFT_1 = PathPlanner.loadPath("left-1", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared);
            
    public MoveAcrossLine()
    {
        addCommands(new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_1).robotRelative(),
        new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_1).robotRelative(),
        new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_1).robotRelative());
    }

    
}
