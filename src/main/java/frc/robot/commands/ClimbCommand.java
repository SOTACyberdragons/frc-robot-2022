package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClimbCommand extends CommandBase {
    private boolean upward;

    boolean vertical;

    public ClimbCommand(boolean upward, boolean vertical) {
        this.vertical = vertical;

        if (vertical) {
            addRequirements(RobotContainer.m_verticalClimber);
        } else {
            addRequirements(RobotContainer.m_horizontalClimber);
        }

        this.upward = upward;
    }

    public void initialize() {}

    @Override
    public void execute() {
        if (vertical) {
            RobotContainer.m_verticalClimber.spin(this.upward);
        } else {
            RobotContainer.m_horizontalClimber.spin(this.upward);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_verticalClimber.stop();
    }
}
