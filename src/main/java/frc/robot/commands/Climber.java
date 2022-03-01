package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class Climber extends CommandBase {

    public Climber() {
        addRequirements(Robot.m_climber);
    }

    public void initialize() {

    }

    public void execute() {
        Robot.m_climber.moveUp();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        Robot.m_climber.stop();
    }

}
