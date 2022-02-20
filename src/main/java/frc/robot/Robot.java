package frc.robot;

import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.vision.*;

import frc.robot.commands.DifferentialDriveWithJoysticks;
import frc.robot.grip.CustomPipeline;
import frc.robot.grip.GripPipeline;
import frc.robot.subsystems.ShooterTest;
import frc.robot.utils.MultiplexedDistanceSensor;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    public static RobotContainer m_robotContainer;
    public static ShooterTest m_shooterTest;

    public static MultiplexedDistanceSensor m_leftSensor;
    public static MultiplexedDistanceSensor m_rightSensor;

    @Override
    public void robotInit() {

        RobotContainer.m_robotDrive.zeroHeading();
        RobotContainer.m_robotDrive.resetEncoders();

        RobotContainer.m_robotDrive.resetOdometry(RobotContainer.m_robotDrive.getPose());

        m_robotContainer = new RobotContainer();

        CommandScheduler.getInstance();
        CommandScheduler.getInstance().enable();

        // Reset the Falcon encoders
        RobotContainer.m_robotDrive.leftMaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        RobotContainer.m_robotDrive.leftSlave.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        RobotContainer.m_robotDrive.rightMaster.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        RobotContainer.m_robotDrive.rightSlave.setSelectedSensorPosition(0, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);

        m_leftSensor = new MultiplexedDistanceSensor(I2C.Port.kOnboard, 7);
        m_rightSensor = new MultiplexedDistanceSensor(I2C.Port.kOnboard, 6);

        m_leftSensor.setAutomaticMode(true);
        m_rightSensor.setAutomaticMode(true);
        m_leftSensor.setDistanceUnits(Unit.kMillimeters);
        m_rightSensor.setDistanceUnits(Unit.kMillimeters);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Left Range", m_leftSensor.getRange());
        SmartDashboard.putNumber("Right Range", m_rightSensor.getRange());
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().enable();

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
