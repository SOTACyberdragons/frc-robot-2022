//                                                 @
//                                                  &@@
//                          * .                    * @@@
//                           * (@   ,                 @@@@
//                               @@@*       /          @@@@
//                                @@@@@@    @@(     ,* ,@@@@@
//                         %@@@@/*  @@@@@@@@       ,**. @@@@@@
//                      #********,    @@@@@@@@@@    ***  @@@@@@
//                   **********    /    @@@@@@@@@@@@   ,  @@@@@@
//                              &@@/  (@  (@@@@@@@@@@@@   @@@@@@@
//                            @@@@@//  @@@@@@@@@@@@@@@@@@& @@@@@@@
//                          @@@@@@@//  @@@@@@@@# .@@@@@@@@@@@@@@@@
//                         @@@@@@&///  %@@@@@@@@(  *  @@@@@@@@@@@@
//                       *@@@@@//   @@@@@@@@@@@@@@%     @@@@@@@@@@@
//                      .@@@@@@@@@@//   .@@@@@@@@@@@@@@  @@@@@@@@@@@
//                      @@@@@@@@@@@@@@(/     @@@@@@@@@@@@@@@@@@@@@@@@@
//                   @ %@@@@@@@@@@@@@@   ,  @@@@@@@@@@@@@@@@@@@@@@@@@@@
//                  @@ @@@@@@@@@@@@@   .             *@@@@@@@@@  @@@@@@#
//                 @@@ @@@@@@@@@@@@%   *******@@@&///     &@@@@@@@@@@@@@
//                 @**  @@@@@@@@@@@   ******@@@@@@,          @@@@@@@@@@
//                 #*** @@@@@@@@@@@   *****@@@@@                  @@@@*
//                ***   @@@@@@@@@@@  ,****@@@,
//                 *      @@@@@@@@@@.  *****@@
//                          @@@@@@@@@#   ***%@
//                           ,@@@@@@@@@    ***@,  /
//                              @@@@@@@@@(    ***   //////*.     */
//                                 //@@@@@@%/      *    ///////
//                                 @    //////////
//                                   @@**
//                                       @*****
//                                             *

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

public class AutoMiddle extends SequentialCommandGroup {

    private static final Trajectory MIDDLE_1 =
            PathPlanner.loadPath(
                    "middle-1",
                    Constants.kMaxSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared);
    private static final Trajectory MIDDLE_2 =
            PathPlanner.loadPath(
                    "middle-2",
                    Constants.kMaxSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared,
                    true);
    private static final Trajectory MIDDLE_3 =
            PathPlanner.loadPath(
                    "middle-3",
                    Constants.kMaxSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared);

    public AutoMiddle() {

        // TODO See if changing paths to Robot relative works better
        // RobotContainer.m_drive.resetOdometry(MIDDLE_1.getInitialPose());

        addCommands(
                new ParallelDeadlineGroup(
                        new DrivetrainRamseteCommand(RobotContainer.m_drive, MIDDLE_1)
                                .robotRelative()
                                .robotRelative(),
                        new SpinIntake()),
                new ParallelDeadlineGroup(
                        new DrivetrainRamseteCommand(RobotContainer.m_drive, MIDDLE_2),
                        new SpinUpShooter("High")),
                new ParallelDeadlineGroup(new WaitCommand(1), new ShootCargo("High")),
                new ParallelDeadlineGroup(
                        new DrivetrainRamseteCommand(RobotContainer.m_drive, MIDDLE_3),
                        new SpinIntake()),
                new ParallelDeadlineGroup(new WaitCommand(1), new ShootCargo("High")));
    }
}
