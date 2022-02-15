/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package frc.robot.commands;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;

import frc.robot.subsystems.DrivetrainRefactored;
import frc.robot.utils.TrajectoryLoader;

public class DrivetrainRamsete extends RamseteCommand {

    private boolean resetPosition;
    private Trajectory trajectory;
    private  DrivetrainRefactored drivetrain;

    public DrivetrainRamsete(DrivetrainRefactored drivetrain, Trajectory trajectory) {
        super(
                trajectory,
                drivetrain::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(
                    Constants.ksVolts,
                    Constants.kvVoltSecondsPerMeter,
                    Constants.kaVoltSecondsSquaredPerMeter),
                Constants.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                new PIDController(Constants.kPDriveVel, 0, 0),
                new PIDController(Constants.kPDriveVel, 0, 0),
                drivetrain::tankDriveVolts,
                drivetrain);


        this.resetPosition = true;
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
    }

    public DrivetrainRamsete(DrivetrainRefactored drivetrain, String path) {
        this(drivetrain, TrajectoryLoader.getTrajectory(path));
    }

    public DrivetrainRamsete(DrivetrainRefactored drivetrain, String... paths) {
        this(drivetrain, TrajectoryLoader.getTrajectory(paths));
    }

    // [DEFAULT] Resets the drivetrain to the begining of the trajectory
    public DrivetrainRamsete robotRelative() {
        this.resetPosition = true;
        return this;
    }

    // Make the trajectory relative to the field
    public DrivetrainRamsete fieldRelative() {
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
