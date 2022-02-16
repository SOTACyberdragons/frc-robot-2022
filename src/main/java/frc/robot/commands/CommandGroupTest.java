/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package frc.robot.commands;


import frc.robot.RobotContainer;
import frc.robot.Robot;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;



public class CommandGroupTest extends SequentialCommandGroup {

    private static final String FIRST_PATH =
            "PathWeaver/output/firstPath.wpilib.json";

    /** Creates a new FourBallAuton. */
    public CommandGroupTest(RobotContainer robot) {
        // Starting up subsystems
        addCommands(
                new DrivetrainRamsete(RobotContainer.m_robotDrive, SnakePath.trajectory()),
                new WaitCommand(1),
                new DriveForward(3)
        );
    }
}