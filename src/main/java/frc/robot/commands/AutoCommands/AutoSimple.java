package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SpinIntake; 

public class AutoSimple extends SequentialCommandGroup{
    private static final Trajectory LEFT_1 = PathPlanner.loadPath("left-1", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared);
    public AutoSimple (){
        addCommands(
            new ParallelDeadlineGroup(
                        new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_1)
                                .robotRelative(),
                        new SpinIntake()),
        )
    }
    
}

