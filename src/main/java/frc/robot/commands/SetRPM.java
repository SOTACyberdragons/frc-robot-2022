// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// THIS WAS A HUGE WASTE OF TIME! DOING IT WITH THE FALCONS INSTEAD.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SetRPM extends CommandBase {

    // PID Constansts
    private static final double kP = .025; // Power
    private static final double kI = 0; // Ease in sensitivity
    private static final double kD = 0; // Smoothing
    // private static final double kF = 0.2; // Feed forward

    public static final double kSVolts = 0.25;
    public static final double kVVoltSecondsPerRotation = 0.037894;

    // private final SimpleMotorFeedforward m_Feedforward = new SimpleMotorFeedforward(kSVolts, kVVoltSecondsPerRotation);

    public PIDController m_shooterPIDController = new PIDController(kP, kI, kD);

    // RPM Values
    public static double targetRPM = 8;

    /** Creates a new SetRPM. */
    public SetRPM() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_robotDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooterPIDController.setSetpoint(targetRPM);
        m_shooterPIDController.setTolerance(.5, .5);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentRPM = (RobotContainer.m_robotDrive.getWheelRPM());
        double pidOutput = m_shooterPIDController.calculate(currentRPM);
        RobotContainer.m_robotDrive.m_drive(pidOutput, 0);

        SmartDashboard.putNumber("Target RPM: ", targetRPM);
        SmartDashboard.putNumber("PID Output: ", pidOutput);
        SmartDashboard.putNumber("Set point: ", m_shooterPIDController.getSetpoint());

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_robotDrive.m_drive(0, 0);
        RobotContainer.m_controller.setRumble(RumbleType.kLeftRumble, 0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // m_shooterPIDController.atSetpoint();
    }
}
