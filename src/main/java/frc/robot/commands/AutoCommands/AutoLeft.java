// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootCargo;
import frc.robot.commands.SpinIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoLeft extends SequentialCommandGroup {

    private static final Trajectory LEFT_1 = PathPlanner.loadPath("left-1", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared);
    private static final Trajectory LEFT_2 = PathPlanner.loadPath("left-2", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared, true);
    private static final Trajectory LEFT_3 = PathPlanner.loadPath("left-3", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared);
    private static final Trajectory LEFT_4 = PathPlanner.loadPath("left-4", Constants.kMaxSpeedMetersPerSecond,
            Constants.kMaxAccelerationMetersPerSecondSquared);

    public AutoLeft(RobotContainer robotContainer) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_1),
                        new SpinIntake()),
                new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_2),
                        new SpinUpShooter("High")),
                new ParallelDeadlineGroup(new WaitCommand(1), new ShootCargo("Low")),
                new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_3),
                        new SpinIntake()),
                new ParallelDeadlineGroup(new WaitCommand(1), new ShootCargo("Low")),
                new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.m_drive, LEFT_4),
                        new SpinIntake()));
    }
}
