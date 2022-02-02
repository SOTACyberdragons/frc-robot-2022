package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.spinWithGyro;

public class TestGyroCommand extends CommandBase
{
    spinWithGyro gyroSubsystem = new spinWithGyro();
    
    public void initialize() {
        System.out.println("Gyro is initialized\n");
        
        Robot.m_robotDrive.gyro.reset();
    }
    
    public void execute() {
        gyroSubsystem.spin();
    }
    
    @Override
    public boolean isFinished()
    {
        System.out.println(Robot.m_robotDrive.getAngle());

        if (Robot.m_robotDrive.gyro.getAngle() >= 90 || 
        Robot.m_robotDrive.gyro.getAngle() <= -90) 
        {
            return true;
        }
        
        return false;
    }
    
    // @Override
    // public void end(boolean interrupted)
    // {
        //     gyroSubsystem.stop();
        // }
    }