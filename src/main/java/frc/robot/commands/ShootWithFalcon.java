// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FalconShooter;

public class ShootWithFalcon extends CommandBase {
    public static double motorPower;
    
    FalconShooter m_shooter = new FalconShooter();

    /** Creates a new ShootWithFalcon. */
    public ShootWithFalcon(double targetPower) {
        motorPower = targetPower;
        // Use addRequirements() here to declare subsystem dependencies
        // addRequirements(m_shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooter._rightMaster.set(TalonFXControlMode.PercentOutput, motorPower);
        m_shooter._leftMaster.set(TalonFXControlMode.PercentOutput, motorPower);
        System.out.println(motorPower);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("Shooter Velocity: ", m_shooter.getVelocity());
        SmartDashboard.putNumber("Feedforward: ", m_shooter.getFeedforward());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter._rightMaster.set(TalonFXControlMode.PercentOutput, 0);
        m_shooter._leftMaster.set(TalonFXControlMode.PercentOutput, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
