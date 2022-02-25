package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;


public class TalonExample extends CommandBase {

    public TalonExample()
    {
        addRequirements(Robot.m_shooterTest);
    }

    public void initialize()
    {
        
    }

    public void execute()
    {
        Robot.m_shooterTest.givePower();
    }

    @Override
    public boolean isFinished() {
            return false;
    }
    
     // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        Robot.m_shooterTest.stop();
    }
    
}