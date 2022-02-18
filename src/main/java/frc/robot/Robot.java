package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DifferentialDriveWithJoysticks;
import frc.robot.subsystems.ShooterTest;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    public static RobotContainer m_robotContainer;
    public static ShooterTest m_shooterTest;

    @Override
    public void robotInit() {
        RobotContainer.m_robotDrive.zeroHeading();
        RobotContainer.m_robotDrive.resetEncoders();

        RobotContainer.m_robotDrive.resetOdometry(RobotContainer.m_robotDrive.getPose());

        m_robotContainer = new RobotContainer();

        CommandScheduler.getInstance().enable();

        // Reset the Falcon encoders
        RobotContainer.m_robotDrive.leftMaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        RobotContainer.m_robotDrive.leftSlave.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        RobotContainer.m_robotDrive.rightMaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        RobotContainer.m_robotDrive.rightSlave.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().enable();

        RobotContainer.m_robotDrive.zeroHeading();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().enable();

        RobotContainer.m_robotDrive.zeroHeading();
        RobotContainer.m_robotDrive.resetEncoders();

        CommandScheduler.getInstance().setDefaultCommand(RobotContainer.m_robotDrive,
                new DifferentialDriveWithJoysticks());
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().enable();
    }

    @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();
    }

}
