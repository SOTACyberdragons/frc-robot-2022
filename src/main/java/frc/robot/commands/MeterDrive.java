package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class MeterDrive extends CommandBase {
    private double meters = 0;

    public MeterDrive(int meterCount)
    {
        this.meters = meterCount;
    }

    public void initialize()
    {
        Robot.m_robotDrive.zeroEncoders();
    }

    public void execute()
    {
        Robot.m_robotDrive.drive(0.6, 0);
    }

    @Override
    public boolean isFinished()
    {
        if (Robot.m_robotDrive.getDistance() >= meters) {
            return true;
        } else {
            return false;
        }
    }
}
