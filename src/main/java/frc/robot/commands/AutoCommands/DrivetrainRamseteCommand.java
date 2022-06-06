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

/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.TrajectoryLoader;

public class DrivetrainRamseteCommand extends RamseteCommand {

    protected boolean resetPosition;
    protected Trajectory trajectory;
    protected Drivetrain drivetrain;

    public DrivetrainRamseteCommand(Drivetrain drivetrain, Trajectory trajectory) {
        super(
                trajectory,
                drivetrain::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(
                        Constants.ksDrivetrain, Constants.kvDrivetrain, Constants.kaDrivetrain),
                Constants.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                new PIDController(Constants.kPDrivetrain, 0, Constants.kDDrivetrain),
                new PIDController(Constants.kPDrivetrain, 0, Constants.kDDrivetrain),
                drivetrain::tankDriveVolts,
                drivetrain);

        this.resetPosition = false;
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
    }

    public DrivetrainRamseteCommand(Drivetrain drivetrain, String path) {
        this(drivetrain, TrajectoryLoader.getTrajectory(path));
    }

    public DrivetrainRamseteCommand(Drivetrain drivetrain, String... paths) {
        this(drivetrain, TrajectoryLoader.getTrajectory(paths));
    }

    // [DEFAULT] Resets the drivetrain to the begining of the trajectory
    public DrivetrainRamseteCommand robotRelative() {
        this.resetPosition = true;
        return this;
    }

    // Make the trajectory relative to the field
    public DrivetrainRamseteCommand fieldRelative() {
        this.resetPosition = false;
        return this;
    }

    @Override
    public void initialize() {
        super.initialize();

        if (resetPosition) {
            drivetrain.resetOdometry(trajectory.getInitialPose());
        }
    }
}
