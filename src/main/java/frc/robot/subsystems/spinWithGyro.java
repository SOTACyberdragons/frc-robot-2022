package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class spinWithGyro extends SubsystemBase {
    void zeroGyro()
    {
        Robot.m_robotDrive.gyro.addYaw(-Robot.m_robotDrive.gyro.getYaw());
    }

    public spinWithGyro()
    {

    }

    public void spin()
    {
        // zeroGyro();

        // while (Robot.m_robotDrive.gyro.getYaw() < 181) {
        //     Robot.m_robotDrive.tankDriveVolts(0.5, -05);
        // }
        
        Robot.m_robotDrive.drive(0.5, -0.5);
    }

    // public void stop()
    // {
    //     Robot.m_robotDrive.leftSlave.set(ControlMode.PercentOutput, 0);
    //     Robot.m_robotDrive.rightSlave.set(ControlMode.PercentOutput, 0);
    // }
}
