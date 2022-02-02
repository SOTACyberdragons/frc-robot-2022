package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.globalDriveState;

/** An example command that uses an example subsystem. */
public class DifferentialDriveWithJoysticks extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DifferentialDriveWithJoysticks() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    globalDriveState.update++;
    
    double throttle = 1;
    Robot.m_robotDrive.drive(-RobotContainer.getLeftJoyY()*throttle, RobotContainer.getRightJoyX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
