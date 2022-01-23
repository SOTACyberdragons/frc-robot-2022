package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterTest;

public class ShootTest extends CommandBase {
    private ShooterTest test = new ShooterTest();

    public ShootTest()
    {
        addRequirements(test);
    }

    public void initialize()
    {
        
    }

    public void execute()
    {
        test.givePower();
    }

    @Override 
    public boolean isFinished()
    {
        return false;
    }

    public void end()
    {

    }
}
