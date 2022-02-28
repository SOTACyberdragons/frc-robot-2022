// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.SimpleShooter;

public class SpinShooter extends CommandBase {
    // Motor characterization
    private static double kS = 0;
    private static double kV = 0;
    private static double kA = 0;

    // Motor PID values
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;

    private double velocityTolerance = 50;
    private static SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA);

    // Initialize the PID Controller
    public PIDController m_pidController = new PIDController(kP, kI, kD);

    // Expose input variables
    private static double shooterRPM = 0;

    /** Creates a new SpinShooter. */
    public SpinShooter(double targetRPM, SimpleShooter m_shoote) {
        shooterRPM = targetRPM;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_pidController.setSetpoint(shooterRPM);
        m_pidController.setTolerance(0, velocityTolerance);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // double pidOutput = m_pidController.calculate(Robot.m_simpleShooter.getRPM() + feedForward.calculate(shooterRPM));
        double pidOutput = m_pidController.calculate(Robot.m_simpleShooter.getRPM(), shooterRPM) + feedForward.calculate(shooterRPM);
        Robot.m_simpleShooter.setPower(pidOutput);

        if (m_pidController.atSetpoint()) {
            System.out.println("At set point!");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.m_simpleShooter.setPower(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
