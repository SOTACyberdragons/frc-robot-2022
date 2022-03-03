// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AimTarget extends CommandBase {

    // Motor characterization
    private final double kS = 0.8975;
    private final double kV = 0.0026249;
    private final double kA = 8.3993E-05;

    // Motor PID values
    private final double kP = 0.1084;
    private final double kI = 0;
    private final double kD = 0.0039948;

    double desiredYaw;
    double errorYaw;
    double rotationSpeed;
    boolean turningRight;

    PIDController turnController = new PIDController(kP, 0, kD);

    /** Creates a new AimTarget. */
    public AimTarget(double targetYaw) {
        desiredYaw = targetYaw + RobotContainer.m_drive.getRotation();
        errorYaw = desiredYaw - RobotContainer.m_drive.getRotation();

        if (targetYaw > 0) {
            turningRight = true;
        } else {
            turningRight = false;
        }
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // -1.0 required to ensure positive PID controller effort _increases_ ya
        rotationSpeed = -turnController.calculate(errorYaw, 0);
        RobotContainer.m_drive.arcadeDrive(0, rotationSpeed);
        errorYaw = desiredYaw - RobotContainer.m_drive.getRotation();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_drive.arcadeDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (turningRight & errorYaw < 1) {
            return true;
        } else if (!turningRight & errorYaw > -1) {
            return true;
        } else {
            return false;
        }
    }
}
