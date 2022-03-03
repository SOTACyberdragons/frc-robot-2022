// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class MotionMagic extends CommandBase {

    private double desiredAngle;

    /** Creates a new MotionMagic. */
    public MotionMagic(double inputAngle) {
        desiredAngle = inputAngle;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    /**
     * determine whether two numbers are "approximately equal" by seeing if they
     * are within a certain "tolerance percentage," with `tolerancePercentage` given
     * as a percentage (such as 10.0 meaning "10%").
     *
     * @param tolerancePercentage 1 = 1%, 2.5 = 2.5%, etc.
     */
    public static boolean motionTolerance(double desiredValue, double actualValue, double tolerancePercentage) {
        double diff = Math.abs(desiredValue - actualValue);
        double tolerance = tolerancePercentage / 100 * desiredValue; 
        return diff < tolerance; 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.m_drive.setAngle(desiredAngle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return motionTolerance(desiredAngle, RobotContainer.m_drive.getAngle(), 5.0);
    }
}
