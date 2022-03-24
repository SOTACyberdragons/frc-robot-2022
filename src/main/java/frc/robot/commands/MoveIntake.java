package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class MoveIntake extends CommandBase {
    private String up;

    public MoveIntake(String up)
    {
        this.up = up;
    }

    @Override
    public void initialize() { 
        RobotContainer.m_intake.moveIntake(up);        
        
    } 

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {        
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        
    }
}
