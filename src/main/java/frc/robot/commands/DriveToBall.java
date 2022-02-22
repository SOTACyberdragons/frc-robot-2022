package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToBall extends CommandBase {
    public static boolean reachedBall;
    
    @Override
    public void initialize()
    {
        reachedBall = false;
    }
}
