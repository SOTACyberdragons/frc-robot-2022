package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AutoDistance;

public class DriveAutoDistance extends CommandBase {
    private AutoDistance autoDistance;

    public void initialize() 
    {
        autoDistance = new AutoDistance();

        addRequirements(autoDistance);
    }

    public void execute()
    {
        //autoDistance.driveOnPath();
    }
}
